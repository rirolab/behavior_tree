import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MoveJoint, MoveParallel
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Dual-arm real-hardware test job that replays configured joint pose presets.
    """

    ACTION_TIMEOUT_SEC = 5.0

    def __init__(self, node):
        super(Move, self).__init__(node)
        self.init_blackboard_parameters()

    def acceptable_step(self, step):
        """
        Check whether this job should accept a grounding step for this primitive action.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        if step.get("primitive_action") != "real_drb_dual_test":
            return False
        elif not self.check_robot_count(step, num_robot_required=2):
            return False
        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable step is well-formed enough to
        keep the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        if not self.acceptable_step(step):
            return StepValidationResult.NOT_APPLICABLE

        grounded_robot_names = make_string_list(step.get("robot", []))
        left_robot, right_robot = self.resolve_left_right_robots(step)
        bt_robot_names = getattr(self._node, "robot_names", [])

        if len(grounded_robot_names) != 2 or len(set(grounded_robot_names)) != 2:
            return StepValidationResult.REJECT_GOAL
        if left_robot is None or right_robot is None or left_robot == right_robot:
            return StepValidationResult.REJECT_GOAL
        if bt_robot_names and (
            left_robot not in bt_robot_names or right_robot not in bt_robot_names
        ):
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
                "real_drb_dual_test_job: rejecting new goal, previous still in the pipeline"
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

    def resolve_left_right_robots(self, step):
        """
        Resolve the grounded left/right robot names from one step.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`tuple`: resolved ``(left_robot, right_robot)`` names.
        """
        grounded_robot_names = make_string_list(step.get("robot", []))
        left_robot = next(
            (robot_name for robot_name in grounded_robot_names if "left" in robot_name),
            None,
        )
        right_robot = next(
            (robot_name for robot_name in grounded_robot_names if "right" in robot_name),
            None,
        )
        return left_robot, right_robot

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
            raise RuntimeError("real_drb_dual_test_job: robot_names must be provided")

        if not self.acceptable_step(goal[idx]):
            return None

        step = goal[idx]
        left_robot, right_robot = self.resolve_left_right_robots(step)
        if left_robot is None or right_robot is None or left_robot == right_robot:
            raise RuntimeError(
                "real_drb_dual_test_job: expected one left robot and one right robot"
            )
        if left_robot not in action_client or right_robot not in action_client:
            raise RuntimeError(
                "real_drb_dual_test_job: missing action client for one or more robots"
            )

        global_blackboard = py_trees.blackboard.Client()
        global_blackboard.register_key(
            key="pose_presets",
            access=py_trees.common.Access.READ,
        )

        base_start = global_blackboard.pose_presets.get("base_start")
        if base_start is None:
            raise RuntimeError(
                "real_drb_dual_test_job: missing pose preset [base_start]"
            )
        base_start_left_joint_goal = base_start.get("left_joint_pos")
        base_start_right_joint_goal = base_start.get("right_joint_pos")
        if base_start_left_joint_goal is None or base_start_right_joint_goal is None:
            raise RuntimeError(
                "real_drb_dual_test_job: missing left/right joint preset in [base_start]"
            )

        stack_side_start = global_blackboard.pose_presets.get("stack_side_start")
        if stack_side_start is None:
            raise RuntimeError(
                "real_drb_dual_test_job: missing pose preset [stack_side_start]"
            )
        stack_side_start_left_joint_goal = stack_side_start.get("left_joint_pos")
        stack_side_start_right_joint_goal = stack_side_start.get("right_joint_pos")
        if (
            stack_side_start_left_joint_goal is None
            or stack_side_start_right_joint_goal is None
        ):
            raise RuntimeError(
                "real_drb_dual_test_job: missing left/right joint preset in [stack_side_start]"
            )

        above_mold_start = global_blackboard.pose_presets.get("above_mold_start")
        if above_mold_start is None:
            raise RuntimeError(
                "real_drb_dual_test_job: missing pose preset [above_mold_start]"
            )
        above_mold_start_left_joint_goal = above_mold_start.get("left_joint_pos")
        above_mold_start_right_joint_goal = above_mold_start.get("right_joint_pos")
        if (
            above_mold_start_left_joint_goal is None
            or above_mold_start_right_joint_goal is None
        ):
            raise RuntimeError(
                "real_drb_dual_test_job: missing left/right joint preset in [above_mold_start]"
            )

        base_start_parallel = MoveParallel.MoveParallel(name="BaseStartParallel")
        base_start_parallel.add_children(
            [
                MoveJoint.MOVEJ(
                    name=f"{left_robot}_BaseStart",
                    action_client=action_client[left_robot],
                    action_goal=base_start_left_joint_goal,
                    timeout=self.ACTION_TIMEOUT_SEC,
                    robot_name=left_robot,
                ),
                MoveJoint.MOVEJ(
                    name=f"{right_robot}_BaseStart",
                    action_client=action_client[right_robot],
                    action_goal=base_start_right_joint_goal,
                    timeout=self.ACTION_TIMEOUT_SEC,
                    robot_name=right_robot,
                ),
            ]
        )

        stack_side_start_parallel = MoveParallel.MoveParallel(
            name="StackSideStartParallel"
        )
        stack_side_start_parallel.add_children(
            [
                MoveJoint.MOVEJ(
                    name=f"{left_robot}_StackSideStart",
                    action_client=action_client[left_robot],
                    action_goal=stack_side_start_left_joint_goal,
                    timeout=self.ACTION_TIMEOUT_SEC,
                    robot_name=left_robot,
                ),
                MoveJoint.MOVEJ(
                    name=f"{right_robot}_StackSideStart",
                    action_client=action_client[right_robot],
                    action_goal=stack_side_start_right_joint_goal,
                    timeout=self.ACTION_TIMEOUT_SEC,
                    robot_name=right_robot,
                ),
            ]
        )

        above_mold_start_parallel = MoveParallel.MoveParallel(
            name="AboveMoldStartParallel"
        )
        above_mold_start_parallel.add_children(
            [
                MoveJoint.MOVEJ(
                    name=f"{left_robot}_AboveMoldStart",
                    action_client=action_client[left_robot],
                    action_goal=above_mold_start_left_joint_goal,
                    timeout=self.ACTION_TIMEOUT_SEC,
                    robot_name=left_robot,
                ),
                MoveJoint.MOVEJ(
                    name=f"{right_robot}_AboveMoldStart",
                    action_client=action_client[right_robot],
                    action_goal=above_mold_start_right_joint_goal,
                    timeout=self.ACTION_TIMEOUT_SEC,
                    robot_name=right_robot,
                ),
            ]
        )

        root = py_trees.composites.Sequence(name="RealDrbDualTest", memory=True)
        root.add_children(
            [
                base_start_parallel,
                stack_side_start_parallel,
                above_mold_start_parallel,
            ]
        )
        return root
