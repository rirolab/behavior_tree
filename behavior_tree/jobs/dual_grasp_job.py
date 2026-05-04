import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MovePose, RingWorldModel
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Dual-arm grasp job that moves the holding robot and approach robot
    to the ring regrasp target poses from RingWorldModel.
    """

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
        if step.get("primitive_action") != "dual_grasp":
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

        # Validate that the requested robot roles exist in this tree.
        bt_robot_names = getattr(self._node, "robot_names", [])
        holding_robot = step.get("holding_robot")
        approach_robot = step.get("approach_robot")

        if not holding_robot or not approach_robot:
            return StepValidationResult.REJECT_GOAL
        if holding_robot == approach_robot:
            return StepValidationResult.REJECT_GOAL
        if holding_robot not in bt_robot_names or approach_robot not in bt_robot_names:
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
                "dual_grasp_job: rejecting new goal, previous still in the pipeline"
            )
        else:
            # Cache the full grounding if any step belongs to this job.
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

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        if robot_names is None:
            assert "robot_names must be provided as a parameter or argument to create_root"

        if not self.acceptable_step(goal[idx]):
            return None
        
        # Read the current command and normalize the robot list.
        step = goal[idx]
        grounded_robots = make_string_list(step.get("robot", []))

        # Resolve robot roles and common execution parameters.
        holding_robot = step["holding_robot"]
        approach_robot = step["approach_robot"]
        action_clients = action_client
        plan_name = "Plan" + idx
        move_timeout = float(step.get("move_timeout", step.get("move_timeout_sec", 3.0)))

        # Estimate the regrasp target poses for both robots.
        pose_estimator = RingWorldModel.POSE_ESTIMATOR(
            name=plan_name,
            object_dict={},
            robot_names=robot_names,
            holding_robot=holding_robot,
            approach_robot=approach_robot,
            tf_buffer=kwargs["tf_buffer"],
        )

        # Move the holding robot to the upper regrasp target pose.
        move_holding = MovePose.MOVEP(
            name=f"{holding_robot}_MoveRegraspUp",
            action_client=action_clients[holding_robot],
            action_goal={"pose": plan_name + "/regrasp_target_up"},
            timeout=move_timeout,
            robot_name=holding_robot,
        )

        # Move the approach robot to the lower regrasp target pose.
        move_approach = MovePose.MOVEP(
            name=f"{approach_robot}_MoveRegraspDown",
            action_client=action_clients[approach_robot],
            action_goal={"pose": plan_name + "/regrasp_target_down"},
            timeout=move_timeout,
            robot_name=approach_robot,
        )

        # Execute pose estimation first, then the two MoveP actions in sequence.
        root = py_trees.composites.Sequence(name="DualGrasp", memory=True)
        root.add_children([pose_estimator, move_holding, move_approach])
        return root
