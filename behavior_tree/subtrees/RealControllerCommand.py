import json
import time

import py_trees
from riro_srvs.srv import StringString


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

            # Forward the command as-is so the bridge owns scope parsing and validation.
            command = dict(self.resolve_command())
            req = StringString.Request()
            req.data = json.dumps(command)
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
                self.feedback_message = f"real controller command failed: {exception}"
                return py_trees.common.Status.FAILURE

            response = self.future.result()
            if response is None:
                self.feedback_message = "real controller command returned no response"
                return py_trees.common.Status.FAILURE

            # Decode the bridge's JSON envelope and cache it for later debugging.
            try:
                payload = json.loads(response.data or "{}")
            except json.JSONDecodeError:
                self.feedback_message = "real controller bridge returned invalid JSON"
                return py_trees.common.Status.FAILURE

            if not isinstance(payload, dict):
                self.feedback_message = (
                    "real controller bridge returned an invalid payload"
                )
                return py_trees.common.Status.FAILURE

            self.result_status = "succeeded" if bool(payload.get("success")) else "aborted"
            self.result_message = str(payload.get("message", "") or "")
            self.result_payload = dict(payload.get("payload") or {})

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
