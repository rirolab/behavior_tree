"""Behavior-tree leaf for commands handled by the G1 locomotion client."""

from __future__ import annotations

import json
import time
import uuid

from action_msgs.msg import GoalStatus
import py_trees
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from riro_srvs.srv import StringGoalStatus


class LocomotionCommand(py_trees.behaviour.Behaviour):
    """Send one command through the dedicated G1 locomotion client."""

    def __init__(
        self,
        name,
        action_type,
        timeout,
        command_client,
        goal_status_topic,
    ):
        super().__init__(name=name)
        self.action_type = action_type
        self.timeout = float(timeout)
        self._client = command_client
        self._goal_status_topic = str(goal_status_topic)
        self._future = None
        self._goal_uuid = None
        self._status = GoalStatus.STATUS_UNKNOWN
        self._deadline = None
        self._subscription = None

    def setup(self, node):
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._subscription = node.create_subscription(
            GoalStatus,
            self._goal_status_topic,
            self._status_callback,
            qos,
        )

    def _status_callback(self, message):
        if self._goal_uuid is None:
            return
        if list(message.goal_info.goal_id.uuid) == self._goal_uuid:
            self._status = int(message.status)

    def initialise(self):
        self._future = None
        self._goal_uuid = list(uuid.uuid4().bytes)
        self._status = GoalStatus.STATUS_UNKNOWN
        self._deadline = time.monotonic() + self.timeout

    def update(self):
        if not self._client.service_is_ready():
            if time.monotonic() >= self._deadline:
                self.feedback_message = "locomotion command service unavailable"
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        if self._future is None:
            request = StringGoalStatus.Request()
            request.data = json.dumps(
                {
                    "action_type": self.action_type,
                    "uuid": self._goal_uuid,
                    "timeout": self.timeout,
                    "enable_wait": False,
                }
            )
            self._future = self._client.call_async(request)
            return py_trees.common.Status.RUNNING

        if self._future.done():
            try:
                response_status = int(self._future.result().goal_status.status)
            except Exception as exc:
                self.feedback_message = f"locomotion service failed: {exc}"
                return py_trees.common.Status.FAILURE
            if response_status in (
                GoalStatus.STATUS_UNKNOWN,
                GoalStatus.STATUS_ABORTED,
                GoalStatus.STATUS_CANCELED,
            ):
                self.feedback_message = "locomotion command rejected"
                return py_trees.common.Status.FAILURE

        if self._status == GoalStatus.STATUS_SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if self._status in (
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELED,
        ):
            return py_trees.common.Status.FAILURE
        if time.monotonic() >= self._deadline:
            self.feedback_message = "locomotion goal timed out"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING
