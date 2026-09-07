import json

import numpy as np
import py_trees
from action_msgs.msg import GoalStatus

from riro_srvs.srv import StringGoalStatus

from . import Policy


# Blackboard namespace for the single dual-arm policy node. Mirrors the
# per-robot "{robot}/goal_id" / "{robot}/goal_status" namespacing used by
# Move.MOVE, but here a single dual node owns one goal-status channel
# ("dual_arm_client/goal_status"), so we read from "dual/goal_id" /
# "dual/goal_status" instead of any per-arm namespace.
DUAL_NAMESPACE = "dual"


class MOVEBYPOLICYDUAL(Policy.MOVEBYPOLICY):
    """
    Execute a single dual-arm policy through the dual_arm_client.

    Near-identical to :class:`Policy.MOVEBYPOLICY` (the request still uses
    ``action_type == "moveByPolicy"``), except the goal status is read from the
    shared dual blackboard keys ``dual/goal_id`` / ``dual/goal_status`` rather
    than the per-robot ``{robot}/goal_id`` / ``{robot}/goal_status`` keys. The
    ``action_client`` passed in must be the dual service client (a
    ``riro_srvs/srv/StringGoalStatus`` client on ``dual_arm_client/command``).
    """

    def __init__(self, name, action_client, action_goal=None, timeout=5.0, robot_name=None):
        """
        Initialise a dual-arm policy execution behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            action_client (:class:`~rclpy.client.Client`): dual command client.
            action_goal (:obj:`dict`): policy command payload.
            timeout (:obj:`float`): command timeout in seconds.
            robot_name (:obj:`str`): unused; the dual node is not per-robot.
        """
        super(MOVEBYPOLICYDUAL, self).__init__(
            name=name,
            action_client=action_client,
            action_goal=action_goal,
            timeout=timeout,
            robot_name=robot_name,
        )

        # Re-attach the goal-status blackboard client under the fixed "dual"
        # namespace so current_goal_id()/current_goal_status()/
        # goal_matches_blackboard() (inherited from Move.MOVE) resolve to
        # "dual/goal_id" / "dual/goal_status", matching the dual ToBlackboard
        # writer in multi_dynamic_behavior_tree.create_root().
        self.blackboard = self.attach_blackboard_client(
            name=self.name,
            namespace=DUAL_NAMESPACE,
        )
        self.blackboard.register_key(
            key=self.goal_id_key,
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key=self.goal_status_key,
            access=py_trees.common.Access.READ,
        )
        self.logger.debug("%s.__init__()" % self.__class__.__name__)


def create_subtree(action_client, step_goal, **kwargs):
    """
    Create a dual-arm policy execution subtree for one grounding step.

    Args:
        action_client (:class:`~rclpy.client.Client`): dual command client.
        step_goal (:obj:`dict`): policy command payload (``skill_id`` + ``timeout``).

    Returns:
       :class:`~py_trees.behaviour.Behaviour`: subtree root
    """
    root = py_trees.composites.Sequence(name="PolicyDual", memory=True)
    run_policy = MOVEBYPOLICYDUAL(
        name="MoveByPolicyDual",
        action_client=action_client,
        action_goal=step_goal,
        timeout=float(step_goal.get("timeout", 5.0)),
        robot_name=kwargs.get("robot_name"),
    )
    root.add_child(run_policy)
    return root
