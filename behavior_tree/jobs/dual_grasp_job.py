import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MoveParallel, MovePose, RingWorldModel, MoveJoint, Gripper
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
        move_timeout_short = float(step.get("move_timeout_short_sec", 1.0))

        # Find the left/right arm names for the horizontal top grasp move.
        left_robot = next((robot for robot in grounded_robots if "left" in robot), None)
        right_robot = next((robot for robot in grounded_robots if "right" in robot), None)
        if left_robot is None or right_robot is None:
            raise RuntimeError("dual_grasp_job: expected one left robot and one right robot in the grounding")
        if holding_robot not in [left_robot, right_robot] or approach_robot not in [left_robot, right_robot]:
            raise RuntimeError("dual_grasp_job: expected holding and approach robots to be the left and right robots in the grounding")

        # Get the blackboard and parameters
        approach_robot_blackboard = py_trees.blackboard.Client(namespace=approach_robot)
        approach_robot_blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        approach_robot_blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
        approach_robot_blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
        approach_robot_blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)
        approach_robot_blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)
        holding_robot_blackboard = py_trees.blackboard.Client(namespace=holding_robot)
        holding_robot_blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        holding_robot_blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
        holding_robot_blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
        holding_robot_blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)
        holding_robot_blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)
        left_robot_blackboard = approach_robot_blackboard if left_robot == approach_robot else holding_robot_blackboard
        right_robot_blackboard = approach_robot_blackboard if right_robot == approach_robot else holding_robot_blackboard

        # Estimate the regrasp target poses for both robots.
        pose_estimator = RingWorldModel.POSE_ESTIMATOR(
            name=plan_name,
            object_dict={'target': 'dual_grasp_target'},
            robot_names=robot_names,
            holding_robot=holding_robot,
            approach_robot=approach_robot,
            tf_buffer=kwargs["tf_buffer"],
        )

        # Move the approach robot to the lower regrasp target pose.
        move_approach_seq = py_trees.composites.Sequence(name="MoveApproachSeq", memory=True)
        move_approach_right_parallel = MoveParallel.MoveParallel(name="ApproachRobotRegraspDownRightParallel")
        move_holding = MovePose.MOVEP(
            name=f"HoldingRobotRegraspUp",
            action_client=action_clients[holding_robot],
            action_goal={"pose": plan_name + "/regrasp_target_up"},
            timeout=move_timeout,
            robot_name=holding_robot,
        )
        move_approach_right = MovePose.MOVEP(
            name=f"ApproachRobotRegraspDownRight",
            action_client=action_clients[approach_robot],
            action_goal={"pose": plan_name + "/regrasp_target_down_right"}, # TODO: currenlty predeifined
            timeout=move_timeout,
            robot_name=approach_robot,
        )
        move_approach_right_open = Gripper.GOTO(
            name="ApproachRobotGripperOpen",
            action_client=action_clients[approach_robot],
            action_goal=approach_robot_blackboard.gripper_open_pos,
            force=approach_robot_blackboard.gripper_open_force,
            timeout=1,
            robot_name=approach_robot
        )
        move_approach_right_parallel.add_children([move_holding, move_approach_right, move_approach_right_open])
        move_approach = MovePose.MOVEP(
            name=f"ApproachRobotRegraspDown",
            action_client=action_clients[approach_robot],
            action_goal={"pose": plan_name + "/regrasp_target_down"},
            timeout=move_timeout_short,
            robot_name=approach_robot,
        )
        move_approach_close = Gripper.GOTO(
            name="ApproachRobotGripperClose",
            action_client=action_clients[approach_robot],
            action_goal=approach_robot_blackboard.gripper_close_pos,
            force=approach_robot_blackboard.gripper_close_force,
            timeout=1,
            robot_name=approach_robot
        )
        move_approach_seq.add_children([move_approach_right_parallel, move_approach, move_approach_close])

        # Move both arms to their horizontal grasp top poses in parallel with waypoints
        move_horizontal_wp = MoveParallel.MoveParallel(name="HorizontalGraspTopWP")
        move_horizontal_right_wp_seq = py_trees.composites.Sequence(name="MoveHorizontalGraspTopRightWPSeq", memory=True)
        move_horizontal_right_wp1 = MovePose.MOVEP(
            name=f"{right_robot}_MoveHorizontalGraspTopRightWP1",
            action_client=action_clients[right_robot],
            action_goal={"pose": plan_name + "/horizontal_grasp_top_right_wp1"},
            timeout=move_timeout/2,
            robot_name=right_robot,
        )
        move_horizontal_right_wp2 = MovePose.MOVEP(
            name=f"{right_robot}_MoveHorizontalGraspTopRightWP2",
            action_client=action_clients[right_robot],
            action_goal={"pose": plan_name + "/horizontal_grasp_top_right_wp2"},
            timeout=move_timeout/2,
            robot_name=right_robot,
        )
        move_horizontal_right_wp_seq.add_children([move_horizontal_right_wp1, move_horizontal_right_wp2])
        move_horizontal_left_wp2 = MovePose.MOVEP(
            name=f"{left_robot}_MoveHorizontalGraspTopLeftWP2",
            action_client=action_clients[left_robot],
            action_goal={"pose": plan_name + "/horizontal_grasp_top_left_wp2"},
            timeout=move_timeout,
            robot_name=left_robot,
        )
        move_horizontal_wp.add_children([move_horizontal_right_wp_seq, move_horizontal_left_wp2])

        # Move both arms to their horizontal grasp top poses in parallel.
        move_horizontal = MoveParallel.MoveParallel(name="HorizontalGraspTop")
        move_horizontal_right = MovePose.MOVEP(
            name=f"{right_robot}_MoveHorizontalGraspTopRight",
            action_client=action_clients[right_robot],
            action_goal={"pose": plan_name + "/horizontal_grasp_top_right"},
            timeout=move_timeout,
            robot_name=right_robot,
        )
        move_horizontal_left = MovePose.MOVEP(
            name=f"{left_robot}_MoveHorizontalGraspTopLeft",
            action_client=action_clients[left_robot],
            action_goal={"pose": plan_name + "/horizontal_grasp_top_left"},
            timeout=move_timeout,
            robot_name=left_robot,
        )
        move_horizontal.add_children([move_horizontal_right, move_horizontal_left])

        # Open together
        open_together = MoveParallel.MoveParallel(name="OpenTogether")
        open_together_right = Gripper.GOTO(
            name="OpenTogetherRight",
            action_client=action_clients[right_robot],
            action_goal=right_robot_blackboard.gripper_open_pos,
            force=right_robot_blackboard.gripper_open_force,
            timeout=1,
            robot_name=right_robot
        )
        open_together_left = Gripper.GOTO(
            name="OpenTogetherLeft",
            action_client=action_clients[left_robot],
            action_goal=left_robot_blackboard.gripper_open_pos,
            force=left_robot_blackboard.gripper_open_force,
            timeout=1,
            robot_name=left_robot
        )
        open_together.add_children([open_together_right, open_together_left])

        # Execute pose estimation first, then the two MoveP actions, then the
        # horizontal top grasp MoveP actions in parallel.
        root = py_trees.composites.Sequence(name="DualGrasp", memory=True)
        # root.add_children([s_init1, pose_estimator, move_holding, move_approach, move_horizontal])
        root.add_children([pose_estimator, 
                           move_approach_seq,
                           move_horizontal_wp, 
                           move_horizontal,
                           open_together])
        return root
