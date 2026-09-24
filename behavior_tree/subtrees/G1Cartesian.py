"""G1 Cartesian command leaves and stand-then-move composition."""

from __future__ import annotations

import json

from action_msgs.msg import GoalStatus
import numpy as np
import py_trees
from riro_srvs.srv import StringGoalStatus

from behavior_tree.subtrees import G1Locomotion, Move, MoveParallel


STAND_TRANSITION_DEADLINE_SECONDS = 30.0


class MoveCartesian(Move.MOVE):
    """Send one TCP pose to a G1 Cartesian complex action client."""

    def update(self):
        if self.cmd_req is None:
            self.feedback_message = "Cartesian action client is unavailable"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16, dtype=np.uint8)
            request = StringGoalStatus.Request()
            request.data = json.dumps(
                self._make_command(
                    "moveCartesian",
                    self.action_goal,
                    uuid=self.goal_uuid_des.tolist(),
                    enable_wait=False,
                )
            )
            self.future = self.cmd_req.call_async(request)
            self.sent_goal = True
            self.feedback_message = "Sending a Cartesian TCP goal"
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
        self.feedback_message = "running"
        return py_trees.common.Status.RUNNING


def create_subtree(
    action_clients,
    target_poses,
    robot_names,
    locomotion_client,
    locomotion_status_topic,
    timeout,
    name="G1StandCartesian",
):
    """Request standing mode, then command the selected TCP poses in parallel."""
    timeout = float(timeout)
    hold_stand = G1Locomotion.LocomotionCommand(
        name="G1HoldStand",
        action_type="holdStand",
        timeout=STAND_TRANSITION_DEADLINE_SECONDS,
        command_client=locomotion_client,
        goal_status_topic=locomotion_status_topic,
    )
    arm_commands = [
        MoveCartesian(
            name=f"Cartesian_{robot_name}",
            action_client=action_clients[robot_name],
            action_goal={"target_pose": target_poses[robot_name]},
            timeout=timeout,
            robot_name=robot_name,
        )
        for robot_name in robot_names
    ]
    move_arms = MoveParallel.MoveParallel(
        name="G1MoveTCPs", children=arm_commands
    )
    root = py_trees.composites.Sequence(name=name, memory=True)
    root.add_children([hold_stand, move_arms])
    return root
