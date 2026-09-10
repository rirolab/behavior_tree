import json
import threading
import time
from urllib import error
from urllib import request

import py_trees

from behavior_tree.transition_trace import elapsed_seconds, trace_event


class RUN_POLICY(py_trees.behaviour.Behaviour):
    """
    Send one blocking policy-server run request without blocking the BT tick loop.
    """

    def __init__(self, name, policy, server_url, timeout, endpoint="/run_policy"):
        super(RUN_POLICY, self).__init__(name=name)

        # Validate the static request fields before the first tick.
        if not isinstance(policy, str) or not policy.strip():
            raise ValueError(f"{self.name}: policy must be a non-empty string")
        if not isinstance(server_url, str) or not server_url.strip():
            raise ValueError(f"{self.name}: server_url must be a non-empty string")
        if not isinstance(timeout, (int, float)) or float(timeout) <= 0.0:
            raise ValueError(f"{self.name}: timeout must be positive")

        # Store immutable HTTP request settings.
        self.policy = policy.strip()
        self.server_url = server_url.rstrip("/")
        self.timeout = float(timeout)
        self.endpoint = "/" + str(endpoint).strip().strip("/")

        # Store per-request thread state.
        self.deadline = None
        self.thread = None
        self.result = None
        self.exception = None
        self.request_started_perf = None
        self.timeout_traced = False

    def initialise(self):
        # Reset state and launch the blocking HTTP request in a worker thread.
        self.deadline = time.monotonic() + self.timeout
        self.result = None
        self.exception = None
        self.request_started_perf = time.perf_counter()
        self.timeout_traced = False
        trace_event(
            "bt",
            "policy_http.thread_start",
            name=self.name,
            policy=self.policy,
            server_url=self.server_url,
            timeout_sec=self.timeout,
        )
        self.thread = threading.Thread(
            target=self._request_policy,
            name=f"{self.name}_http_request",
            daemon=True,
        )
        self.thread.start()
        self.feedback_message = f"requested policy {self.policy}"

    def update(self):
        # Return the worker result once the HTTP call finishes.
        if self.exception is not None:
            self.feedback_message = f"policy request failed: {self.exception}"
            return py_trees.common.Status.FAILURE
        if self.result is not None:
            if bool(self.result.get("success", False)):
                elapsed = float(self.result.get("elapsed_sec", 0.0))
                self.feedback_message = f"policy {self.policy} succeeded in {elapsed:.1f}s"
                return py_trees.common.Status.SUCCESS
            self.feedback_message = f"policy {self.policy} returned failure: {self.result}"
            return py_trees.common.Status.FAILURE

        # Cancel and fail when the policy server does not answer before the BT timeout.
        if self.deadline is not None and time.monotonic() >= self.deadline:
            if not self.timeout_traced:
                trace_event(
                    "bt",
                    "policy_http.bt_timeout",
                    name=self.name,
                    policy=self.policy,
                    elapsed_sec=elapsed_seconds(self.request_started_perf),
                    timeout_sec=self.timeout,
                )
                self.timeout_traced = True
            self._request_cancel()
            self.feedback_message = f"policy {self.policy} timed out"
            return py_trees.common.Status.FAILURE

        # Keep ticking while the policy server is still executing the request.
        self.feedback_message = f"waiting for policy {self.policy}"
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Ask the server to cancel if the tree interrupts this running behaviour.
        if new_status == py_trees.common.Status.INVALID and self.thread is not None and self.thread.is_alive():
            trace_event(
                "bt",
                "policy_http.interrupted",
                name=self.name,
                policy=self.policy,
                elapsed_sec=elapsed_seconds(self.request_started_perf),
            )
            self._request_cancel()

    def _request_policy(self):
        # Retry connection setup while the launch-time policy preload is still finishing.
        payload = json.dumps({"policy": self.policy}).encode("utf-8")
        deadline = time.monotonic() + self.timeout + 5.0
        last_exception = None
        attempt = 0
        while time.monotonic() < deadline:
            attempt += 1
            attempt_started_perf = time.perf_counter()
            try:
                trace_event(
                    "bt",
                    "policy_http.send",
                    name=self.name,
                    policy=self.policy,
                    attempt=attempt,
                    url=f"{self.server_url}{self.endpoint}",
                )
                req = request.Request(
                    f"{self.server_url}{self.endpoint}",
                    data=payload,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                with request.urlopen(req, timeout=max(deadline - time.monotonic(), 1.0)) as response:
                    body = response.read().decode("utf-8")
                self.result = json.loads(body) if body else {}
                trace_event(
                    "bt",
                    "policy_http.response",
                    name=self.name,
                    policy=self.policy,
                    attempt=attempt,
                    elapsed_sec=elapsed_seconds(attempt_started_perf),
                    total_elapsed_sec=elapsed_seconds(self.request_started_perf),
                    success=bool(self.result.get("success", False)),
                    response=self.result,
                )
                return
            except error.HTTPError as exc:
                try:
                    body = exc.read().decode("utf-8")
                    self.result = json.loads(body) if body else {"success": False, "error": str(exc)}
                    trace_event(
                        "bt",
                        "policy_http.http_error",
                        name=self.name,
                        policy=self.policy,
                        attempt=attempt,
                        elapsed_sec=elapsed_seconds(attempt_started_perf),
                        status=getattr(exc, "code", None),
                        response=self.result,
                    )
                except Exception:
                    self.exception = exc
                    trace_event(
                        "bt",
                        "policy_http.http_error_decode_failed",
                        name=self.name,
                        policy=self.policy,
                        attempt=attempt,
                        elapsed_sec=elapsed_seconds(attempt_started_perf),
                        error=exc,
                    )
                return
            except error.URLError as exc:
                last_exception = exc
                trace_event(
                    "bt",
                    "policy_http.url_error",
                    name=self.name,
                    policy=self.policy,
                    attempt=attempt,
                    elapsed_sec=elapsed_seconds(attempt_started_perf),
                    error=exc,
                )
                time.sleep(0.5)
            except Exception as exc:
                self.exception = exc
                trace_event(
                    "bt",
                    "policy_http.exception",
                    name=self.name,
                    policy=self.policy,
                    attempt=attempt,
                    elapsed_sec=elapsed_seconds(attempt_started_perf),
                    error=exc,
                )
                return
        self.exception = last_exception or TimeoutError(f"timed out connecting to {self.server_url}")
        trace_event(
            "bt",
            "policy_http.connection_timeout",
            name=self.name,
            policy=self.policy,
            total_elapsed_sec=elapsed_seconds(self.request_started_perf),
            error=self.exception,
        )

    def _request_cancel(self):
        # Best-effort cancellation keeps timeouts from leaving a policy loop active.
        cancel_started_perf = time.perf_counter()
        trace_event("bt", "policy_http.cancel_send", name=self.name, policy=self.policy)
        try:
            req = request.Request(
                f"{self.server_url}/cancel",
                data=b"{}",
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            request.urlopen(req, timeout=1.0).close()
            trace_event(
                "bt",
                "policy_http.cancel_done",
                name=self.name,
                policy=self.policy,
                elapsed_sec=elapsed_seconds(cancel_started_perf),
            )
        except Exception:
            trace_event(
                "bt",
                "policy_http.cancel_failed",
                name=self.name,
                policy=self.policy,
                elapsed_sec=elapsed_seconds(cancel_started_perf),
            )


class PREPARE_POLICY(RUN_POLICY):
    """Ready the selected policy's camera stream before controller switching."""

    def __init__(self, name, policy, server_url, timeout):
        super(PREPARE_POLICY, self).__init__(
            name=name,
            policy=policy,
            server_url=server_url,
            timeout=timeout,
            endpoint="/prepare_policy",
        )


class RUN_WITH_CLEANUP(py_trees.composites.Composite):
    """Run a policy-control body, then attempt every safety cleanup in order.

    Unlike a normal Sequence, cleanup still runs when the body or an earlier
    cleanup child fails.  The composite returns the original body status only
    after cleanup completes, and reports failure if any cleanup itself failed.
    """

    def __init__(self, name, body, cleanup_children):
        cleanup = list(cleanup_children)
        if not cleanup:
            raise ValueError(f"{name}: cleanup_children must not be empty")
        super(RUN_WITH_CLEANUP, self).__init__(name=name, children=[body, *cleanup])
        self.body = body
        self.cleanup_children = cleanup
        self.body_status = None
        self.cleanup_index = 0
        self.cleanup_failed = False
        self.phase = "body"

    def initialise(self):
        self.body_status = None
        self.cleanup_index = 0
        self.cleanup_failed = False
        self.phase = "body"
        self.current_child = self.body

    def tick(self):
        # Match py_trees composite initialisation semantics before entering either phase.
        if self.status != py_trees.common.Status.RUNNING:
            for child in self.children:
                if child.status != py_trees.common.Status.INVALID:
                    child.stop(py_trees.common.Status.INVALID)
            self.initialise()

        if self.phase == "body":
            for node in self.body.tick():
                yield node
            if self.body.status == py_trees.common.Status.RUNNING:
                self.status = py_trees.common.Status.RUNNING
                yield self
                return
            self.body_status = self.body.status
            self.phase = "cleanup"

        # Attempt every cleanup child even when an earlier one reports FAILURE.
        while self.cleanup_index < len(self.cleanup_children):
            cleanup_child = self.cleanup_children[self.cleanup_index]
            self.current_child = cleanup_child
            for node in cleanup_child.tick():
                yield node
            if cleanup_child.status == py_trees.common.Status.RUNNING:
                self.status = py_trees.common.Status.RUNNING
                yield self
                return
            if cleanup_child.status != py_trees.common.Status.SUCCESS:
                self.cleanup_failed = True
            self.cleanup_index += 1

        final_status = self.body_status
        if self.cleanup_failed or final_status not in {
            py_trees.common.Status.SUCCESS,
            py_trees.common.Status.FAILURE,
        }:
            final_status = py_trees.common.Status.FAILURE
        self.stop(final_status)
        yield self
