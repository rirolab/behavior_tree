"""Dex1 commands for one or both G1 hands through the arm CACs."""

from __future__ import annotations

import json

from action_msgs.msg import GoalStatus
import numpy as np
import py_trees
from riro_srvs.srv import StringGoalStatus

from behavior_tree.subtrees import Move, MoveParallel


class MoveGripper(Move.MOVE):
    """Send a Dex1 target and wait for the CAC's final published command."""

    def __init__(self, name, action_client, target_q, timeout, robot_name):
        super().__init__(
            name=name,
            action_client=action_client,
            action_goal={"target_q": target_q},
            timeout=timeout,
            robot_name=robot_name,
            goal_channel="gripper",
        )

    def update(self):
        if self.cmd_req is None:
            self.feedback_message = "Dex1 action client is unavailable"
            return py_trees.common.Status.FAILURE
        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16, dtype=np.uint8)
            request = StringGoalStatus.Request()
            request.data = json.dumps(
                self._make_command(
                    "moveGripper",
                    self.action_goal,
                    uuid=self.goal_uuid_des.tolist(),
                    enable_wait=False,
                )
            )
            self.future = self.cmd_req.call_async(request)
            self.sent_goal = True
            self.feedback_message = "Sending a Dex1 target"
            return py_trees.common.Status.RUNNING

        response_status = self.command_response_status()
        if response_status is not None:
            return response_status
        if self.current_goal_id() is None or not self.goal_matches_blackboard():
            return py_trees.common.Status.RUNNING
        status = self.current_goal_status()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            return py_trees.common.Status.SUCCESS
        if status in (
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_CANCELED,
        ):
            self.feedback_message = "FAILURE"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING


def create_subtree(action_clients, robot_names, target_q, timeout, name):
    commands = [
        MoveGripper(
            name=f"Dex1_{robot_name}",
            action_client=action_clients[robot_name],
            target_q=target_q,
            timeout=timeout,
            robot_name=robot_name,
        )
        for robot_name in robot_names
    ]
    return MoveParallel.MoveParallel(name=name, children=commands)
