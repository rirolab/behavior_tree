import ast
import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MoveJoint


DUAL_ACTIONS = {
    "dual_init",
    "dual_move_joint",
    "dual_movej",
    "dual_joint",
}


def _list_parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    value = node.get_parameter(name).value
    if isinstance(value, str):
        value = ast.literal_eval(value)
    return [float(v) for v in value]


class Move(base_job.BaseJob):
    def __init__(self, node):
        super(Move, self).__init__(node)

        self.blackboard.register_key(key="left_init_config", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="right_init_config", access=py_trees.common.Access.WRITE)
        self.blackboard.left_init_config = _list_parameter(
            self._node,
            "left_init_config",
            [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0],
        )
        self.blackboard.right_init_config = _list_parameter(
            self._node,
            "right_init_config",
            [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0],
        )

    def incoming(self, msg):
        if self.goal:
            self._node.get_logger().error("dual_arm_job: rejecting new goal, previous still in the pipeline")
            return

        grounding = json.loads(msg.data)["params"]
        for i in range(len(grounding.keys())):
            action = grounding[str(i + 1)].get("primitive_action", "")
            if action in DUAL_ACTIONS:
                self.goal = grounding
                break

    @staticmethod
    def create_root(action_client, idx="1", goal=std_msgs.Empty(), **kwargs):
        command = dict(goal[idx])
        nested_goal = command.get("goal")
        if isinstance(nested_goal, str):
            nested_goal = json.loads(nested_goal)
        if isinstance(nested_goal, dict):
            merged = dict(nested_goal)
            merged.update(command)
            command = merged

        action = command.get("primitive_action", command.get("action", ""))
        if action not in DUAL_ACTIONS:
            return None

        blackboard = py_trees.blackboard.Client()
        blackboard.register_key(key="left_init_config", access=py_trees.common.Access.READ)
        blackboard.register_key(key="right_init_config", access=py_trees.common.Access.READ)

        if action == "dual_init":
            positions = list(blackboard.left_init_config) + list(blackboard.right_init_config)
        else:
            positions = Move._extract_positions(command)

        timeout = float(command.get("timeout", command.get("timeout_sec", 3.0)))
        root = py_trees.composites.Sequence(name="DualArm", memory=True)
        root.add_child(
            MoveJoint.MOVEJ(
                name="DualMoveJoint",
                action_client=action_client,
                action_goal=positions,
                timeout=timeout,
            )
        )
        return root

    @staticmethod
    def _extract_positions(command):
        positions = command.get("positions")
        if positions is not None:
            if len(positions) != 14:
                raise ValueError("positions must contain 14 values for dual-arm commands")
            return [float(value) for value in positions]

        left = Move._first_present(
            command,
            ("left", "left_positions", "left_joint_positions", "left_config", "arm_l"),
        )
        right = Move._first_present(
            command,
            ("right", "right_positions", "right_joint_positions", "right_config", "arm_r"),
        )
        if left is None or right is None:
            raise KeyError("dual command requires left/right joint positions or a 14-value positions list")
        if len(left) != 7 or len(right) != 7:
            raise ValueError("left and right positions must each contain 7 values")
        return [float(value) for value in left] + [float(value) for value in right]

    @staticmethod
    def _first_present(command, keys):
        for key in keys:
            if key in command:
                return command[key]
        return None
