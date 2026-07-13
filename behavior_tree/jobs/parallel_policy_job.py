import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MoveParallel, Policy
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


# Robots a parallel-policy step must name (order-insensitive).
PARALLEL_ARM_ROBOTS = {"left_arm", "right_arm"}


class Move(base_job.BaseJob):
    """
    Job handler for running TWO independent single-arm policies concurrently.

    Each arm runs its OWN policy (its own checkpoint) on its OWN arm_client via
    the per-arm ``moveByPolicy`` path, the two wrapped in a parallel composite so
    they execute simultaneously. This differs from dual_policy_job, which drives
    BOTH arms from ONE 16-dim model through the shared dual node. No controller
    switching is performed (ffw_bg2 publishes to broadcasters directly).

    Grounding step schema::

        {
          "primitive_action": "parallel_policy_execute",
          "robot": ["left_arm", "right_arm"],
          "left_arm":  {"skill_id": "<left skill_id>",  "timeout": 30.0},
          "right_arm": {"skill_id": "<right skill_id>", "timeout": 30.0}
        }
    """

    def __init__(self, node):
        super(Move, self).__init__(node)

    def acceptable_step(self, step):
        """
        Check whether this job should accept a grounding step for this primitive action.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        if step.get("primitive_action") != "parallel_policy_execute":
            return False
        elif not self.check_robot_count(step, num_robot_required=2):
            return False
        else:
            return True

    def _robot_skill_id(self, step, robot_name):
        """Read a robot's policy skill_id from its per-arm block, or None."""
        robot_block = step.get(robot_name) or {}
        skill_id = robot_block.get("skill_id")
        return skill_id if bool(skill_id) else None

    def validate_step(self, step):
        """
        Validate whether an acceptable parallel-policy step is well-formed enough
        to keep the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        # Ignore steps that are not parallel policy executions.
        if step.get("primitive_action") != "parallel_policy_execute":
            return StepValidationResult.NOT_APPLICABLE

        # The action matches: both arms must be named and each must carry its own
        # skill_id, otherwise the step is malformed and rejects the goal.
        robot_names = make_string_list(step.get("robot", []))
        if set(robot_names) != PARALLEL_ARM_ROBOTS:
            return StepValidationResult.REJECT_GOAL
        for robot_name in robot_names:
            if self._robot_skill_id(step, robot_name) is None:
                return StepValidationResult.REJECT_GOAL

        return StepValidationResult.ACCEPT_GOAL

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            self._node.get_logger().error(
                "parallel_policy_job: rejecting new goal, previous still in the pipeline"
            )
        else:
            grounding = json.loads(msg.data)["params"]
            for i in range(len(grounding.keys())):
                step = grounding.get(str(i + 1))
                if step is None:
                    continue
                if self.acceptable_step(step):
                    self.goal = grounding
                    break

    def create_root(
        self,
        action_client,
        idx="1",
        goal=std_msgs.Empty(),
        robot_names=None,
        **kwargs,
    ):
        """
        Create the job subtree based on the incoming goal specification.

        Called by multi_dynamic_behavior_tree's pre_tick_handler with the
        MULTI-robot signature: ``action_client`` is a ``{robot_name: client}``
        mapping. One single-arm ``MOVEBYPOLICY`` is built per robot against that
        robot's own command client and skill_id, and the two are wrapped in a
        ``MoveParallel`` so they run at the same time.

        Args:
            action_client (:obj:`dict`): per-arm command clients keyed by robot.
            idx (:obj:`str`): step index in the grounding plan.
            goal (:obj:`dict`): full grounding plan.
            robot_names ([:obj:`str`]): requested robot names.

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        if not self.acceptable_step(goal[idx]):
            return None

        # Both parallel children dispatch through the central policy manager;
        # they differentiate by goal_id and per-arm blackboard scope, not by
        # which client is called.
        policy_action_client = kwargs.get("policy_action_client")
        if policy_action_client is None:
            self._node.get_logger().error(
                "parallel_policy_job: no policy_action_client provided, cannot build subtree"
            )
            return None

        step = goal[idx]
        grounded_robot_names = make_string_list(step.get("robot", []))

        run_policy_parallel = MoveParallel.MoveParallel(name="RunPolicyParallel")
        for robot_name in grounded_robot_names:
            skill_id = self._robot_skill_id(step, robot_name)
            if skill_id is None:
                self._node.get_logger().error(
                    f"parallel_policy_job: missing skill_id for robot [{robot_name}]"
                )
                return None

            robot_block = step.get(robot_name) or {}
            timeout = float(robot_block.get("timeout", step.get("timeout_sec", 30.0)))
            run_policy = Policy.MOVEBYPOLICY(
                name=f"{robot_name}_MoveByPolicy",
                action_client=policy_action_client,
                action_goal={"skill_id": skill_id, "timeout": timeout},
                timeout=timeout,
                robot_name=robot_name,
            )
            run_policy_parallel.add_child(run_policy)

        return run_policy_parallel
