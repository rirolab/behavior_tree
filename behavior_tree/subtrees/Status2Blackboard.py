import time

import py_trees
import py_trees_ros
from action_msgs.msg import GoalStatus
from py_trees_ros import subscribers

from behavior_tree.utils.debug_file_logger import (
    get_debug_file_logger,
    make_debug_snapshot,
)


class ToBlackboard(subscribers.ToBlackboard):
    def __init__(
        self,
        name,
        topic_name,
        robot_name="",
        goal_channel="arm",
        qos_profile=None,
    ):
        self.robot_name = robot_name
        self.goal_channel = goal_channel
        self._topic_name = topic_name
        self.goal_id_key = (
            f"{robot_name}/{goal_channel}/goal_id" if robot_name else f"{goal_channel}/goal_id"
        )
        self.goal_status_key = (
            f"{robot_name}/{goal_channel}/goal_status"
            if robot_name
            else f"{goal_channel}/goal_status"
        )
        if qos_profile is None:
            qos_profile = py_trees_ros.utilities.qos_profile_unlatched()

        super(ToBlackboard, self).__init__(
            name=name,
            topic_name=topic_name,
            topic_type=GoalStatus,
            blackboard_variables={
                self.goal_id_key: "goal_info.goal_id.uuid",
                self.goal_status_key: "status",
            },
            qos_profile=qos_profile,
        )

        self.blackboard.register_key(
            key=self.goal_id_key,
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key=self.goal_status_key,
            access=py_trees.common.Access.READ,
        )
        self._debug_logger = None
        self._last_debug_snapshot = None
        self._last_debug_snapshot_time = 0.0

    def setup(self, **kwargs):
        # Keep the ROS subscriber setup from py_trees_ros.
        super(ToBlackboard, self).setup(**kwargs)

        # Enable file logging only when the tree node parameter asks for it.
        node = kwargs.get("node")
        enable_debug_file_logging = True
        if node is not None and node.has_parameter("enable_bt_debug_file_logging"):
            enable_debug_file_logging = node.get_parameter(
                "enable_bt_debug_file_logging"
            ).value
            if not isinstance(enable_debug_file_logging, bool):
                raise RuntimeError(
                    f"{self.name}: enable_bt_debug_file_logging must be a bool"
                )
        if enable_debug_file_logging:
            self._debug_logger = get_debug_file_logger("bt", "tree")

    @staticmethod
    def _goal_status_to_string(status):
        if status == GoalStatus.STATUS_ACCEPTED:
            return "ACCEPTED"
        if status == GoalStatus.STATUS_EXECUTING:
            return "EXECUTING"
        if status == GoalStatus.STATUS_CANCELING:
            return "CANCELING"
        if status == GoalStatus.STATUS_SUCCEEDED:
            return "SUCCEEDED"
        if status == GoalStatus.STATUS_CANCELED:
            return "CANCELED"
        if status == GoalStatus.STATUS_ABORTED:
            return "ABORTED"
        if status == GoalStatus.STATUS_UNKNOWN:
            return "UNKNOWN"
        return str(status)

    def _read_blackboard(self, key):
        try:
            return self.blackboard.get(key)
        except KeyError:
            return None

    def _log_snapshot(self, event, **fields):
        # Skip debug file writes when BT debug file logging is disabled.
        if self._debug_logger is None:
            return
        snapshot = make_debug_snapshot(
            {
                "event": event,
                "robot_name": self.robot_name,
                "goal_channel": self.goal_channel,
                "fields": fields,
            }
        )
        now = time.monotonic()
        if (
            snapshot != self._last_debug_snapshot
            or now - self._last_debug_snapshot_time >= 2.0
        ):
            self._last_debug_snapshot = snapshot
            self._last_debug_snapshot_time = now
            self._debug_logger.log(
                event,
                subscriber_name=self.name,
                robot_name=self.robot_name,
                goal_channel=self.goal_channel,
                topic_name=self._topic_name,
                **fields,
            )

    def update(self):
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            goal_id = self._read_blackboard(self.goal_id_key)
            goal_status = self._read_blackboard(self.goal_status_key)
            self._log_snapshot(
                "status2blackboard_update",
                blackboard_write_status=status.name,
                blackboard_goal_id=goal_id,
                blackboard_goal_status=goal_status,
                blackboard_goal_status_name=self._goal_status_to_string(goal_status),
            )
        return status
