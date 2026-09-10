import json
import time

import py_trees
from riro_srvs.srv import StringString

from behavior_tree.transition_trace import elapsed_seconds, trace_event


class REAL_CONTROLLER_COMMAND(py_trees.behaviour.Behaviour):
    """
    Call the real controller-manager bridge and wait for the switch result.
    """

    def __init__(
        self,
        name,
        controller_manager_bridge_service_name="/controller_manager_bridge/switchController",
        command=None,
        timeout=10.0,
    ):
        """
        Initialise a real controller switch behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            controller_manager_bridge_service_name (:obj:`str`): bridge command service.
            command: command dictionary or blackboard key.
            timeout (:obj:`float`): service timeout in seconds.
        """
        super(REAL_CONTROLLER_COMMAND, self).__init__(name=name)
        self.controller_manager_bridge_service_name = str(
            controller_manager_bridge_service_name
        ).strip()
        self.command = command
        self.timeout = float(timeout)
        self.node = None
        self.client = None
        self.sent_goal = False
        self.deadline = None
        self.future = None
        self.request_started_perf = None
        self.timeout_traced = False
        # Track the latest bridge response for BT feedback.
        self.result_status = None
        self.result_message = ""
        self.result_payload = {}
        self.blackboard = self.attach_blackboard_client(name=self.name)
        if isinstance(command, str):
            self.blackboard.register_key(
                key=command,
                access=py_trees.common.Access.READ,
            )

    def setup(self, node):
        """
        Create the real bridge service client.

        Args:
            node (:class:`~rclpy.node.Node`): ROS node that owns communications.
        """
        self.node = node
        # Use one generic switch service and let the bridge resolve arm scopes.
        self.client = node.create_client(
            StringString,
            self.controller_manager_bridge_service_name,
        )

    def initialise(self):
        """
        Reset command state before each tick sequence.
        """
        self.sent_goal = False
        self.deadline = time.monotonic() + self.timeout
        self.future = None
        self.request_started_perf = None
        self.timeout_traced = False
        # Reset the cached bridge response before sending a new request.
        self.result_status = None
        self.result_message = ""
        self.result_payload = {}

    def update(self):
        """
        Send the switch request and report the bridge result.

        Returns:
            :class:`~py_trees.common.Status`: behaviour status.
        """
        if self.client is None:
            self.feedback_message = "real controller bridge client is not initialized"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            if not self.client.wait_for_service(timeout_sec=0.0):
                if time.monotonic() < self.deadline:
                    self.feedback_message = (
                        f"waiting for {self.controller_manager_bridge_service_name}"
                    )
                    return py_trees.common.Status.RUNNING
                self.feedback_message = (
                    f"{self.controller_manager_bridge_service_name} is unavailable"
                )
                return py_trees.common.Status.FAILURE

            # Forward the pre-built real-controller command payload as-is.
            command = dict(self.resolve_command())
            req = StringString.Request()
            req.data = json.dumps(command)
            self.request_started_perf = time.perf_counter()
            trace_event(
                "bt",
                "controller_switch.send",
                name=self.name,
                service=self.controller_manager_bridge_service_name,
                command=command,
                timeout_sec=self.timeout,
            )
            self.future = self.client.call_async(req)
            self.sent_goal = True
            self.feedback_message = (
                f"sent real controller command {command.get('action_type')}"
            )
            return py_trees.common.Status.RUNNING

        # Surface the bridge response in BT feedback when available.
        if self.future is not None and self.future.done():
            # Convert transport- or bridge-level failures into BT failure feedback.
            exception = self.future.exception()
            if exception is not None:
                trace_event(
                    "bt",
                    "controller_switch.exception",
                    name=self.name,
                    service=self.controller_manager_bridge_service_name,
                    elapsed_sec=elapsed_seconds(self.request_started_perf),
                    error=exception,
                )
                self.feedback_message = f"real controller command failed: {exception}"
                return py_trees.common.Status.FAILURE

            response = self.future.result()
            if response is None:
                trace_event(
                    "bt",
                    "controller_switch.no_response",
                    name=self.name,
                    service=self.controller_manager_bridge_service_name,
                    elapsed_sec=elapsed_seconds(self.request_started_perf),
                )
                self.feedback_message = "real controller command returned no response"
                return py_trees.common.Status.FAILURE

            # Decode the bridge's JSON envelope and cache it for later debugging.
            try:
                payload = json.loads(response.data or "{}")
            except json.JSONDecodeError:
                trace_event(
                    "bt",
                    "controller_switch.invalid_json",
                    name=self.name,
                    service=self.controller_manager_bridge_service_name,
                    elapsed_sec=elapsed_seconds(self.request_started_perf),
                    response=response.data,
                )
                self.feedback_message = "real controller bridge returned invalid JSON"
                return py_trees.common.Status.FAILURE

            if not isinstance(payload, dict):
                trace_event(
                    "bt",
                    "controller_switch.invalid_payload",
                    name=self.name,
                    service=self.controller_manager_bridge_service_name,
                    elapsed_sec=elapsed_seconds(self.request_started_perf),
                    response=payload,
                )
                self.feedback_message = (
                    "real controller bridge returned an invalid payload"
                )
                return py_trees.common.Status.FAILURE

            self.result_status = "succeeded" if bool(payload.get("success")) else "aborted"
            self.result_message = str(payload.get("message", "") or "")
            self.result_payload = dict(payload.get("payload") or {})
            trace_event(
                "bt",
                "controller_switch.response",
                name=self.name,
                service=self.controller_manager_bridge_service_name,
                elapsed_sec=elapsed_seconds(self.request_started_perf),
                success=bool(payload.get("success")),
                message=self.result_message,
                payload=self.result_payload,
            )

            if self.result_status == "succeeded":
                requested_profile = str(
                    self.result_payload.get("requested_controller_profile", "") or ""
                )
                if self.result_message:
                    self.feedback_message = (
                        f"real controller command succeeded: {self.result_message}"
                    )
                elif requested_profile:
                    self.feedback_message = (
                        f"real controller command succeeded: {requested_profile}"
                    )
                else:
                    self.feedback_message = "real controller command succeeded"
                return py_trees.common.Status.SUCCESS

            self.feedback_message = (
                f"real controller command failed: {self.result_message}"
                if self.result_message
                else "real controller command failed"
            )
            return py_trees.common.Status.FAILURE

        if time.monotonic() < self.deadline:
            return py_trees.common.Status.RUNNING
        if not self.timeout_traced:
            trace_event(
                "bt",
                "controller_switch.timeout",
                name=self.name,
                service=self.controller_manager_bridge_service_name,
                elapsed_sec=elapsed_seconds(self.request_started_perf),
                timeout_sec=self.timeout,
            )
            self.timeout_traced = True
        self.feedback_message = "real controller command timed out"
        return py_trees.common.Status.FAILURE

    def resolve_command(self):
        """
        Resolve the command from the blackboard or inline command value.

        Returns:
            :obj:`dict`: controller switch command payload.
        """
        if isinstance(self.command, str):
            return self.blackboard.get(self.command)
        return self.command or {}


class CARTESIAN_COMMAND_HTTP_GATE(py_trees.behaviour.Behaviour):
    """
    Send a cartesian command HTTP gate request through a ROS StringString service.
    """

    def __init__(
        self,
        name,
        cartesian_command_http_gate_service_name="/cartesian_command_http_gate",
        command=None,
        timeout=10.0,
    ):
        """
        Initialise a cartesian command HTTP gate behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            cartesian_command_http_gate_service_name (:obj:`str`): gate command service.
            command: command dictionary or blackboard key.
            timeout (:obj:`float`): service timeout in seconds.
        """
        super(CARTESIAN_COMMAND_HTTP_GATE, self).__init__(name=name)

        # Store the service route and command source for this behaviour.
        self.cartesian_command_http_gate_service_name = str(
            cartesian_command_http_gate_service_name
        ).strip()
        self.command = command
        self.timeout = float(timeout)

        # Store ROS client and per-run request state.
        self.node = None
        self.client = None
        self.sent_goal = False
        self.deadline = None
        self.future = None
        self.request_started_perf = None
        self.timeout_traced = False

        # Track the latest gate response for BT feedback.
        self.result_status = None
        self.result_message = ""
        self.result_payload = {}

        # Attach a blackboard client when command points to a runtime key.
        self.blackboard = self.attach_blackboard_client(name=self.name)
        if isinstance(command, str):
            self.blackboard.register_key(
                key=command,
                access=py_trees.common.Access.READ,
            )

    def setup(self, node):
        """
        Create the cartesian command HTTP gate service client.

        Args:
            node (:class:`~rclpy.node.Node`): ROS node that owns communications.
        """
        self.node = node

        # Use StringString so the downstream node can forward JSON to its HTTP path.
        self.client = node.create_client(
            StringString,
            self.cartesian_command_http_gate_service_name,
        )

    def initialise(self):
        """
        Reset command state before each tick sequence.
        """
        # Reset request bookkeeping for the next service call.
        self.sent_goal = False
        self.deadline = time.monotonic() + self.timeout
        self.future = None
        self.request_started_perf = None
        self.timeout_traced = False

        # Reset the cached gate response before sending a new command.
        self.result_status = None
        self.result_message = ""
        self.result_payload = {}

    def update(self):
        """
        Send the cartesian command gate request and report the service result.

        Returns:
            :class:`~py_trees.common.Status`: behaviour status.
        """
        # Fail if setup did not create the ROS service client.
        if self.client is None:
            self.feedback_message = (
                "cartesian command HTTP gate client is not initialized"
            )
            return py_trees.common.Status.FAILURE

        # Send the command once the service is available.
        if not self.sent_goal:
            if not self.client.wait_for_service(timeout_sec=0.0):
                if time.monotonic() < self.deadline:
                    self.feedback_message = (
                        f"waiting for {self.cartesian_command_http_gate_service_name}"
                    )
                    return py_trees.common.Status.RUNNING
                self.feedback_message = (
                    f"{self.cartesian_command_http_gate_service_name} is unavailable"
                )
                return py_trees.common.Status.FAILURE

            # Resolve the command from inline config or a blackboard key.
            if isinstance(self.command, str):
                command = self.blackboard.get(self.command)
            else:
                command = self.command or {}
            if not isinstance(command, dict):
                raise TypeError(
                    f"{self.name}: command should resolve to dict, "
                    f"got {type(command).__name__}"
                )
            command = dict(command)

            # Reject keys outside the cartesian command gate payload.
            allowed_keys = {"reset_mode", "cartesian_command_gate"}
            unknown_keys = set(command.keys()) - allowed_keys
            if unknown_keys:
                raise ValueError(
                    f"{self.name}: unsupported command keys {sorted(unknown_keys)}"
                )

            # Require the gate key and allow only explicit pause/enable commands.
            if "cartesian_command_gate" not in command:
                raise KeyError(f"{self.name}: cartesian_command_gate is required")
            if command["cartesian_command_gate"] not in {"pause", "enable"}:
                raise ValueError(
                    f"{self.name}: cartesian_command_gate should be "
                    "'pause' or 'enable'"
                )

            # Validate reset_mode only when the optional key is present.
            if (
                "reset_mode" in command
                and command["reset_mode"] not in {"pick_reset", "placement_panda"}
            ):
                raise ValueError(
                    f"{self.name}: reset_mode should be "
                    "'pick_reset' or 'placement_panda'"
                )

            # Forward the validated cartesian command gate payload as JSON.
            req = StringString.Request()
            req.data = json.dumps(command)
            self.request_started_perf = time.perf_counter()
            trace_event(
                "bt",
                "cartesian_gate.send",
                name=self.name,
                service=self.cartesian_command_http_gate_service_name,
                command=command,
                timeout_sec=self.timeout,
            )
            self.future = self.client.call_async(req)
            self.sent_goal = True
            self.feedback_message = (
                f"sent cartesian command HTTP gate "
                f"{command.get('cartesian_command_gate')}"
            )
            return py_trees.common.Status.RUNNING

        # Surface the service response in BT feedback when available.
        if self.future is not None and self.future.done():
            exception = self.future.exception()
            if exception is not None:
                trace_event(
                    "bt",
                    "cartesian_gate.exception",
                    name=self.name,
                    service=self.cartesian_command_http_gate_service_name,
                    elapsed_sec=elapsed_seconds(self.request_started_perf),
                    error=exception,
                )
                self.feedback_message = (
                    f"cartesian command HTTP gate failed: {exception}"
                )
                return py_trees.common.Status.FAILURE

            response = self.future.result()
            if response is None:
                trace_event(
                    "bt",
                    "cartesian_gate.no_response",
                    name=self.name,
                    service=self.cartesian_command_http_gate_service_name,
                    elapsed_sec=elapsed_seconds(self.request_started_perf),
                )
                self.feedback_message = (
                    "cartesian command HTTP gate returned no response"
                )
                return py_trees.common.Status.FAILURE

            # Decode the service's JSON envelope and cache it for later debugging.
            try:
                payload = json.loads(response.data or "{}")
            except json.JSONDecodeError:
                trace_event(
                    "bt",
                    "cartesian_gate.invalid_json",
                    name=self.name,
                    service=self.cartesian_command_http_gate_service_name,
                    elapsed_sec=elapsed_seconds(self.request_started_perf),
                    response=response.data,
                )
                self.feedback_message = (
                    "cartesian command HTTP gate returned invalid JSON"
                )
                return py_trees.common.Status.FAILURE

            if not isinstance(payload, dict):
                trace_event(
                    "bt",
                    "cartesian_gate.invalid_payload",
                    name=self.name,
                    service=self.cartesian_command_http_gate_service_name,
                    elapsed_sec=elapsed_seconds(self.request_started_perf),
                    response=payload,
                )
                self.feedback_message = (
                    "cartesian command HTTP gate returned an invalid payload"
                )
                return py_trees.common.Status.FAILURE

            # Convert the service success flag into BT success or failure.
            self.result_status = "succeeded" if bool(payload.get("success")) else "aborted"
            self.result_message = str(payload.get("message", "") or "")
            self.result_payload = dict(payload.get("payload") or {})
            trace_event(
                "bt",
                "cartesian_gate.response",
                name=self.name,
                service=self.cartesian_command_http_gate_service_name,
                elapsed_sec=elapsed_seconds(self.request_started_perf),
                success=bool(payload.get("success")),
                message=self.result_message,
                payload=self.result_payload,
            )

            if self.result_status == "succeeded":
                if self.result_message:
                    self.feedback_message = (
                        f"cartesian command HTTP gate succeeded: "
                        f"{self.result_message}"
                    )
                else:
                    self.feedback_message = "cartesian command HTTP gate succeeded"
                return py_trees.common.Status.SUCCESS

            self.feedback_message = (
                f"cartesian command HTTP gate failed: {self.result_message}"
                if self.result_message
                else "cartesian command HTTP gate failed"
            )
            return py_trees.common.Status.FAILURE

        # Keep running while the service call is still in flight.
        if time.monotonic() < self.deadline:
            return py_trees.common.Status.RUNNING
        if not self.timeout_traced:
            trace_event(
                "bt",
                "cartesian_gate.timeout",
                name=self.name,
                service=self.cartesian_command_http_gate_service_name,
                elapsed_sec=elapsed_seconds(self.request_started_perf),
                timeout_sec=self.timeout,
            )
            self.timeout_traced = True
        self.feedback_message = "cartesian command HTTP gate timed out"
        return py_trees.common.Status.FAILURE
