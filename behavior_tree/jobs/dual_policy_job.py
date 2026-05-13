import copy
import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MoveParallel, Policy
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Dual-arm policy execution job.
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
        if step.get("primitive_action") != "policy_execute":
            return False
        elif not self.check_robot_count(step, num_robot_required=2):
            return False
        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable dual-arm policy step is well-formed enough to
        keep the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        if not self.acceptable_step(step):
            return StepValidationResult.NOT_APPLICABLE

        robot_names = make_string_list(step.get("robot", []))
        if len(robot_names) != 2 or len(set(robot_names)) != 2:
            return StepValidationResult.REJECT_GOAL

        for robot_name in robot_names:
            robot_goal = self.make_robot_goal(step, robot_name)
            if robot_goal is None or not bool(robot_goal.get("skill_id")):
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
                "dual_policy_job: rejecting new goal, previous still in the pipeline"
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

    def make_robot_goal(self, step, robot_name):
        """
        Build the per-robot policy goal from one shared multi-robot step.

        Args:
            step (:obj:`dict`): multi-robot grounding step.
            robot_name (:obj:`str`): robot to build the goal for.

        Returns:
            :obj:`dict`: robot-specific goal, or :obj:`None` if malformed.
        """
        robot_names = make_string_list(step.get("robot", []))
        robot_specific_goal = step.get(robot_name, {})

        if robot_specific_goal is None:
            robot_specific_goal = {}
        elif not isinstance(robot_specific_goal, dict):
            return None

        # Make robot-specific goal
        robot_goal = copy.deepcopy(step)
        for grounded_robot_name in robot_names:
            if grounded_robot_name in robot_goal:
                robot_goal.pop(grounded_robot_name)

        robot_goal.update(copy.deepcopy(robot_specific_goal))
        robot_goal["robot"] = robot_name
        return robot_goal

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

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        if robot_names is None:
            raise RuntimeError("dual_policy_job: robot_names must be provided")

        if not self.acceptable_step(goal[idx]):
            return None

        step = goal[idx]
        grounded_robot_names = make_string_list(step.get("robot", []))

        if len(grounded_robot_names) != 2:
            raise RuntimeError("dual_policy_job: expected exactly two robots in the grounding")

        run_policy_parallel = MoveParallel.MoveParallel(name="RunPolicyParallel")
        for robot_name in grounded_robot_names:
            robot_goal = self.make_robot_goal(step, robot_name)
            if robot_goal is None or not bool(robot_goal.get("skill_id")):
                raise RuntimeError(
                    f"dual_policy_job: missing valid policy goal for robot [{robot_name}]"
                )

            run_policy = Policy.MOVEBYPOLICY(
                name=f"{robot_name}_MoveByPolicy",
                action_client=action_client[robot_name],
                action_goal=robot_goal,
                timeout=float(robot_goal.get("timeout_sec", robot_goal.get("timeout", 5.0))),
                robot_name=robot_name,
            )
            run_policy_parallel.add_child(run_policy)

        root = py_trees.composites.Sequence(name="DualPolicy", memory=True)
        root.add_child(run_policy_parallel)
        return root
