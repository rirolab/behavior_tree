import json
import time

import py_trees

from riro_srvs.srv import StringGoalStatus


def dump_policy_config(robot_goal):
    """
    Serialize the stable policy config without runtime-only routing fields.
    """
    # Drop routing-only fields so equal policy configs map to the same cache key.
    policy_config = {
        key: value
        for key, value in robot_goal.items()
        if key not in {"robot", "step_idx"}
    }
    return json.dumps(policy_config, sort_keys=True)


class LOAD_POLICY_BATCH(py_trees.behaviour.Behaviour):
    """
    Send batch loadPolicy requests and track loaded policies.
    """

    def __init__(self, name, action_clients, policy_requests=None, timeout=60.0):
        super(LOAD_POLICY_BATCH, self).__init__(name=name)

        # Immutable inputs for load_policy behaviour instance.
        self.action_clients = action_clients
        self.policy_requests = list(policy_requests or [])
        self.timeout = float(timeout)

        # Attach the shared blackboard client once at construction time.
        self.blackboard = self.attach_blackboard_client(name=f"{self.name}_bb")
        self.blackboard.register_key(key="preloaded_policies", access=py_trees.common.Access.WRITE)

        # Per-run execution state rebuilt on each initialise().
        self.policy_batches = {}
        self.active_futures = {}
        self.deadline = None
        self.completed_policies = []

    def initialise(self):
        # Reset batch state whenever this behaviour starts again.
        self.policy_batches = {}
        for robot_name, robot_goal in self.policy_requests:
            self.policy_batches.setdefault(robot_name, []).append(robot_goal)
        self.active_futures = {}
        self.deadline = None
        self.completed_policies = []
        self.feedback_message = ""

    def update(self):
        # Exit immediately when there is nothing to preload.
        if not self.policy_batches:
            self.feedback_message = "no policy requests for loadPolicy"
            return py_trees.common.Status.SUCCESS

        # Dispatch one batched load request per robot the first time this behaviour ticks.
        if not self.active_futures:
            for robot_name, robot_goals in self.policy_batches.items():
                self.active_futures[robot_name] = self.action_clients[robot_name].call_async(
                    StringGoalStatus.Request(
                        data=json.dumps(
                            {
                                "action_type": "loadPolicy",
                                "goal": robot_goals,
                                "timeout": self.timeout,
                                "enable_wait": True,
                            }
                        )
                    )
                )
            self.deadline = time.monotonic() + self.timeout
            self.feedback_message = (
                f"loadPolicy requested for "
                f"{sum(len(batch) for batch in self.policy_batches.values())} "
                f"policies on {len(self.policy_batches)} robots"
            )
            return py_trees.common.Status.RUNNING

        # Fold completed robot futures into the shared loaded policy cache.
        failed_robots = []
        completed_robot_names = []
        for robot_name, future in self.active_futures.items():
            if not future.done():
                continue

            try:
                future.result()
            except Exception as error:
                failed_robots.append((robot_name, error))
                continue

            completed_robot_names.append(robot_name)
            for robot_goal in self.policy_batches[robot_name]:
                # Store the dumped policy config so cleanup can track config identity.
                self.completed_policies.append(
                    {
                        "robot": robot_name,
                        "policy_key": dump_policy_config(robot_goal),
                        "step_idx": robot_goal.get("step_idx"),
                        "skill_id": robot_goal.get("skill_id"),
                    }
                )
            self.blackboard.preloaded_policies = list(self.completed_policies)

        # Surface robot batch failures to the parent subtree.
        if failed_robots:
            failed_robot_names = ", ".join(robot_name for robot_name, _ in failed_robots)
            self.feedback_message = f"loadPolicy request failed for {failed_robot_names}"
            return py_trees.common.Status.FAILURE

        # Drop robot futures already merged into the shared cache.
        for robot_name in completed_robot_names:
            del self.active_futures[robot_name]

        # Surface robot batch timeouts to the parent subtree.
        if self.active_futures and time.monotonic() >= self.deadline:
            self.feedback_message = (
                "loadPolicy request timed out for "
                + ", ".join(sorted(self.active_futures.keys()))
            )
            return py_trees.common.Status.FAILURE

        # Finish once every robot batch has completed successfully.
        if not self.active_futures:
            self.feedback_message = "loadPolicy batch finished"
            return py_trees.common.Status.SUCCESS

        # Stay running while at least one robot batch is still in flight.
        self.feedback_message = (
            f"loadPolicy waiting for {len(self.active_futures)} robot batches"
        )
        return py_trees.common.Status.RUNNING

class UNLOAD_POLICY_BATCH(py_trees.behaviour.Behaviour):
    """
    Send batch unloadPolicy requests for currently loaded policies.
    """

    def __init__(self, name, action_clients, timeout=60.0):
        super(UNLOAD_POLICY_BATCH, self).__init__(name=name)

        # Immutable inputs for unload_policy behaviour instance.
        self.action_clients = action_clients
        self.timeout = float(timeout)

        # Attach the shared blackboard client once at construction time.
        self.blackboard = self.attach_blackboard_client(name=f"{self.name}_bb")
        self.blackboard.register_key(key="preloaded_policies", access=py_trees.common.Access.WRITE)

        # Per-run execution state rebuilt on each initialise().
        self.policy_batches = {}
        self.active_futures = {}
        self.deadline = None

    def initialise(self):
        # Snapshot the currently loaded requests at the start of cleanup.
        self.policy_batches = {}
        for policy_request in self.blackboard.preloaded_policies:
            self.policy_batches.setdefault(policy_request["robot"], []).append(policy_request)
        self.active_futures = {}
        self.deadline = None
        self.feedback_message = ""

    def update(self):
        # Exit immediately when there is nothing left to unload.
        if not self.policy_batches:
            self.blackboard.preloaded_policies = []
            self.feedback_message = "no policy requests for unloadPolicy"
            return py_trees.common.Status.SUCCESS

        # Dispatch one batched unload request per robot the first time this behaviour ticks.
        if not self.active_futures:
            for robot_name, robot_policies in self.policy_batches.items():
                self.active_futures[robot_name] = self.action_clients[robot_name].call_async(
                    StringGoalStatus.Request(
                        data=json.dumps(
                            {
                                "action_type": "unloadPolicy",
                                "goal": robot_policies,
                                "timeout": self.timeout,
                                "enable_wait": True,
                            }
                        )
                    )
                )
            self.deadline = time.monotonic() + self.timeout
            self.feedback_message = (
                f"unloadPolicy requested for "
                f"{sum(len(batch) for batch in self.policy_batches.values())} "
                f"policies on {len(self.policy_batches)} robots"
            )
            return py_trees.common.Status.RUNNING

        # Remove completed robot batches from the shared loaded policy cache.
        failed_robots = []
        completed_robot_names = []
        for robot_name, future in self.active_futures.items():
            if not future.done():
                continue

            try:
                future.result()
            except Exception as error:
                failed_robots.append((robot_name, error))
                continue

            completed_robot_names.append(robot_name)
            robot_policy_batch = self.policy_batches[robot_name]
            self.blackboard.preloaded_policies = [
                request
                for request in self.blackboard.preloaded_policies
                if request not in robot_policy_batch
            ]

        # Surface robot batch failures to the parent subtree.
        if failed_robots:
            failed_robot_names = ", ".join(robot_name for robot_name, _ in failed_robots)
            self.feedback_message = f"unloadPolicy request failed for {failed_robot_names}"
            return py_trees.common.Status.FAILURE

        # Drop robot futures already removed from the shared cache.
        for robot_name in completed_robot_names:
            del self.active_futures[robot_name]

        # Surface robot batch timeouts to the parent subtree.
        if self.active_futures and time.monotonic() >= self.deadline:
            self.feedback_message = (
                "unloadPolicy request timed out for "
                + ", ".join(sorted(self.active_futures.keys()))
            )
            return py_trees.common.Status.FAILURE

        # Finish once every robot batch has completed successfully.
        if not self.active_futures:
            self.blackboard.preloaded_policies = []
            self.feedback_message = "unloadPolicy batch finished"
            return py_trees.common.Status.SUCCESS

        # Stay running while at least one robot batch is still in flight.
        self.feedback_message = (
            f"unloadPolicy waiting for {len(self.active_futures)} robot batches"
        )
        return py_trees.common.Status.RUNNING
