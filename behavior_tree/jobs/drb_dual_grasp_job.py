import json

import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MoveParallel, MovePose, RingWorldModel, MoveJoint, Gripper, Wait, Policy
from behavior_tree.subtrees import JointStateLogger
from behavior_tree.subtrees import IsaacSceneCommand
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
        if step.get("primitive_action") != "drb_dual_grasp":
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
        if len(make_string_list(step.get("robot", []))) != 2:
            return StepValidationResult.REJECT_GOAL

        # Require per-robot policy payloads for the final dual policy stage.
        for robot_name in make_string_list(step.get("robot", [])):
            robot_goal = self.make_robot_specific_goal(step, robot_name, step.get("step_idx"))
            if robot_goal is None:
                return StepValidationResult.REJECT_GOAL
            if not bool(robot_goal.get("skill_id")):
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

    def make_move_pose_with_logger(
        self,
        name,
        action_client,
        action_goal,
        robot_name,
        timeout,
        joint_logger_kwargs,
    ):
        """
        Chain one Cartesian move and one joint-state logger in one sequence.
        """
        # Keep move and logger execution strictly ordered.
        move_pose_with_logger = py_trees.composites.Sequence(
            name=f"{name}WithLogger",
            memory=True,
        )

        # Run one Cartesian move with the original behaviour name.
        move_pose = MovePose.MOVEP(
            name=name,
            action_client=action_client,
            action_goal=action_goal,
            robot_name=robot_name,
            timeout=timeout,
        )

        # Dump one joint-state snapshot right after the pose move.
        joint_logger = JointStateLogger.JOINT_STATE_LOGGING(
            name=f"{name}JointLogger",
            **joint_logger_kwargs,
        )

        # Add both stages as one reusable branch.
        move_pose_with_logger.add_children([move_pose, joint_logger])
        return move_pose_with_logger

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
        # move_timeout = float(step.get("move_timeout", step.get("move_timeout_sec", 3.0)))
        # move_timeout_short = float(step.get("move_timeout_short_sec", 1.0))
        GRIPPER_TIME = 0.25
        MOVE_TIME = 0.25
        
        # Find the left/right arm names for the horizontal top grasp move.
        left_robot = next((robot for robot in grounded_robots if "left" in robot), None)
        right_robot = next((robot for robot in grounded_robots if "right" in robot), None)
        if left_robot is None or right_robot is None:
            raise RuntimeError("dual_grasp_job: expected one left robot and one right robot in the grounding")
        if holding_robot not in [left_robot, right_robot] or approach_robot not in [left_robot, right_robot]:
            raise RuntimeError("dual_grasp_job: expected holding and approach robots to be the left and right robots in the grounding")

        # Resolve per-robot policy payloads for the final dual policy stage.
        left_robot_policy_goal = self.make_robot_specific_goal(step, left_robot, idx)
        right_robot_policy_goal = self.make_robot_specific_goal(step, right_robot, idx)
        if left_robot_policy_goal is None or not bool(left_robot_policy_goal.get("skill_id")):
            raise RuntimeError(f"dual_grasp_job: missing valid policy goal for robot [{left_robot}]")
        if right_robot_policy_goal is None or not bool(right_robot_policy_goal.get("skill_id")):
            raise RuntimeError(f"dual_grasp_job: missing valid policy goal for robot [{right_robot}]")

        # Keep one shared joint logger configuration for every pose move.
        joint_logger_kwargs = {
            "warmup_sec": 1.0,
            "joint_states_topic": "/joint_states",
            "output_dir": "/tmp/behavior_tree_joint_logs",
            "left_robot_name": left_robot,
            "right_robot_name": right_robot,
        }

        # Get the blackboard and parameters
        global_blackboard = py_trees.blackboard.Client()
        global_blackboard.register_key(key="pose_presets", access=py_trees.common.Access.READ)
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

        # Resolve base_start joint targets.
        base_start = global_blackboard.pose_presets.get("base_start")
        if base_start is None:
            console.logerror("Pick: Missing pose preset [base_start]")
            return None
        base_start_left_joint_goal = base_start.get("left_joint_pos")
        base_start_right_joint_goal = base_start.get("right_joint_pos")
        if base_start_left_joint_goal is None or base_start_right_joint_goal is None:
            console.logerror("Pick: Missing left/right joint preset in pose preset [base_start]")
            return None

        # Resolve above_mold_start joint targets.
        above_mold_start = global_blackboard.pose_presets.get("above_mold_start")
        if above_mold_start is None:
            console.logerror("Pick: Missing pose preset [above_mold_start]")
            return None
        above_mold_start_left_joint_goal = above_mold_start.get("left_joint_pos")
        above_mold_start_right_joint_goal = above_mold_start.get("right_joint_pos")
        if above_mold_start_left_joint_goal is None or above_mold_start_right_joint_goal is None:
            console.logerror("Pick: Missing left/right joint preset in pose preset [above_mold_start]")
            return None

        # Resolve holding_robot_regrasp_up left joint target.
        holding_robot_regrasp_up = global_blackboard.pose_presets.get("holding_robot_regrasp_up")
        if holding_robot_regrasp_up is None:
            console.logerror("Pick: Missing pose preset [holding_robot_regrasp_up]")
            return None
        holding_robot_regrasp_up_left_joint_goal = holding_robot_regrasp_up.get("left_joint_pos")
        if holding_robot_regrasp_up_left_joint_goal is None:
            console.logerror("Pick: Missing left joint preset in pose preset [holding_robot_regrasp_up]")
            return None
        
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
        # move_holding = self.make_move_pose_with_logger(
        #     name=f"HoldingRobotRegraspUp",
        #     action_client=action_clients[holding_robot],
        #     action_goal={"pose": plan_name + "/regrasp_target_up"},
        #     timeout=MOVE_TIME,
        #     robot_name=holding_robot,
        #     joint_logger_kwargs=joint_logger_kwargs,
        # )
        move_holding_trajectory = MoveJoint.MOVEJT(
            name=f"HoldingRobotRegraspUpTrajectory",
            action_client=action_clients[holding_robot],
            action_goal=[
                base_start_left_joint_goal,
                holding_robot_regrasp_up_left_joint_goal,
            ],
            robot_name=holding_robot,
            timeout=6*MOVE_TIME,
        )
        move_approach_right = self.make_move_pose_with_logger(
            name=f"ApproachRobotRegraspDownRight",
            action_client=action_clients[approach_robot],
            action_goal={"pose": plan_name + "/regrasp_target_down_right"}, # TODO: currenlty predeifined
            timeout=MOVE_TIME,
            robot_name=approach_robot,
            joint_logger_kwargs=joint_logger_kwargs,
        )
        move_approach_right_open = Gripper.GOTO(
            name="ApproachRobotGripperOpen",
            action_client=action_clients[approach_robot],
            action_goal=approach_robot_blackboard.gripper_open_pos,
            force=approach_robot_blackboard.gripper_open_force,
            timeout=MOVE_TIME,
            robot_name=approach_robot
        )
        move_approach_right_parallel.add_children([move_holding_trajectory, move_approach_right, move_approach_right_open])
        move_approach = self.make_move_pose_with_logger(
            name=f"ApproachRobotRegraspDownHalfLeft",
            action_client=action_clients[approach_robot],
            action_goal={"pose": plan_name + "/regrasp_target_down_half_left"},
            timeout=MOVE_TIME,
            robot_name=approach_robot,
            joint_logger_kwargs=joint_logger_kwargs,
        )
        move_approach_wait = Wait.WAIT(
            name="WaitBeforeClose",
            duration=3.0,
            robot_name=approach_robot,
        )
        move_approach_close = Gripper.GOTO(
            name="ApproachRobotGripperClose",
            action_client=action_clients[approach_robot],
            action_goal=approach_robot_blackboard.gripper_close_pos,
            force=approach_robot_blackboard.gripper_close_force,
            timeout=GRIPPER_TIME,
            robot_name=approach_robot
        )
        freeze_approach_finger = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="FreezeFingerJoint",
            command={
                "action_type": "freezeFingerJoint",
                "arm": approach_robot,
                "enabled": True
            },
            timeout=5.0,
        )
        move_approach_seq.add_children([move_approach_right_parallel, move_approach, move_approach_wait, move_approach_close, freeze_approach_finger])

        # Queue above_mold_start as one left/right parallel stage.
        above_mold_start_parallel = MoveParallel.MoveParallel(name="Preset_above_mold_start")
        above_mold_start_parallel.add_children(
            [
                MoveJoint.MOVEJ(
                    name=f"{left_robot}_above_mold_start",
                    action_client=action_clients[left_robot],
                    action_goal=above_mold_start_left_joint_goal,
                    robot_name=left_robot,
                    timeout=4*MOVE_TIME,
                ),
                MoveJoint.MOVEJ(
                    name=f"{right_robot}_above_mold_start",
                    action_client=action_clients[right_robot],
                    action_goal=above_mold_start_right_joint_goal,
                    robot_name=right_robot,
                    timeout=4*MOVE_TIME,
                ),
            ]
        )

        # Move both arms to their horizontal grasp top poses in parallel with waypoints
        # move_horizontal_wp = MoveParallel.MoveParallel(name="HorizontalGraspTopWP")
        # move_horizontal_right_wp_seq = py_trees.composites.Sequence(name="MoveHorizontalGraspTopRightWPSeq", memory=True)
        # move_horizontal_right_wp1 = MovePose.MOVEP(
        #     name=f"{right_robot}_MoveHorizontalGraspTopRightWP1",
        #     action_client=action_clients[right_robot],
        #     action_goal={"pose": plan_name + "/horizontal_grasp_top_right_wp1"},
        #     timeout=3*MOVE_TIME,
        #     robot_name=right_robot,
        # )
        # move_horizontal_right = MovePose.MOVEP(
        #     name=f"{right_robot}_MoveHorizontalGraspTopRight",
        #     action_client=action_clients[right_robot],
        #     action_goal={"pose": plan_name + "/horizontal_grasp_top_right"},
        #     timeout=3*MOVE_TIME,
        #     robot_name=right_robot,
        # )
        # move_horizontal_right_wp_seq.add_children([move_horizontal_right_wp1, move_horizontal_right])
        # move_horizontal_left = MovePose.MOVEP(
        #     name=f"{left_robot}_MoveHorizontalGraspTopLeft",
        #     action_client=action_clients[left_robot],
        #     action_goal={"pose": plan_name + "/horizontal_grasp_top_left"},
        #     timeout=6*MOVE_TIME,
        #     robot_name=left_robot,
        # )
        # move_horizontal_wp.add_children([move_horizontal_right_wp_seq, move_horizontal_left])

        # Open together        
        open_together_seq = py_trees.composites.Sequence(name="MoveHorizontalGraspTopRightWPSeq", memory=True)
        unfreeze_fingers = MoveParallel.MoveParallel(name="OpenTogether")
        unfreeze_finger_left = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="UnfreezeFingerJointLeft",
            command={
                "action_type": "freezeFingerJoint",
                "arm": left_robot,
                "enabled": False
            },
            timeout=5.0,
        )
        unfreeze_finger_right = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="UnfreezeFingerJointRight",
            command={
                "action_type": "freezeFingerJoint",
                "arm": right_robot,
                "enabled": False
            },
            timeout=5.0,
        )
        unfreeze_fingers.add_children([unfreeze_finger_left, unfreeze_finger_right])
        # open_together = MoveParallel.MoveParallel(name="OpenTogether")
        # open_together_right = Gripper.GOTO(
        #     name="OpenTogetherRight",
        #     action_client=action_clients[right_robot],
        #     action_goal=right_robot_blackboard.gripper_open_pos,
        #     force=right_robot_blackboard.gripper_open_force,
        #     timeout=GRIPPER_TIME,
        #     robot_name=right_robot
        # )
        # open_together_left = Gripper.GOTO(
        #     name="OpenTogetherLeft",
        #     action_client=action_clients[left_robot],
        #     action_goal=left_robot_blackboard.gripper_open_pos,
        #     force=left_robot_blackboard.gripper_open_force,
        #     timeout=GRIPPER_TIME,
        #     robot_name=left_robot
        # )
        # open_together.add_children([open_together_right, open_together_left])
        # open_together_seq.add_children([unfreeze_fingers, open_together])

        # Switch both arms to policy controller before dual policy execution.
        dual_switch_controller_in = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="DualSwitchControllerIn",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "cartesian_impedance_controller",
                "target_arms": [left_robot, right_robot],
            },
            timeout=10.0,
        )

        # Run left and right policy goals in parallel at the end of the job.
        run_policy_parallel = MoveParallel.MoveParallel(name="RunPolicyParallel")
        left_run_policy = Policy.MOVEBYPOLICY(
            name=f"{left_robot}_MoveByPolicy",
            action_client=action_clients[left_robot],
            action_goal=left_robot_policy_goal,
            timeout=float(left_robot_policy_goal.get("timeout", 5.0)),
            robot_name=left_robot,
        )
        right_run_policy = Policy.MOVEBYPOLICY(
            name=f"{right_robot}_MoveByPolicy",
            action_client=action_clients[right_robot],
            action_goal=right_robot_policy_goal,
            timeout=float(right_robot_policy_goal.get("timeout", 5.0)),
            robot_name=right_robot,
        )
        run_policy_parallel.add_children([left_run_policy, right_run_policy])

        # Restore joint controller after both policy branches finish.
        dual_switch_controller_out = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="DualSwitchControllerOut",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "joint_trajectory_controller",
                "target_arms": [left_robot, right_robot],
            },
            timeout=10.0,
        )

        # Group controller switching and dual policy execution into one stage.
        dual_policy_seq = py_trees.composites.Sequence(name="DualPolicySeq", memory=True)
        dual_policy_seq.add_children([dual_switch_controller_in, run_policy_parallel, dual_switch_controller_out])

        # Queue both arms back to base_start after the regrasp stage.
        base_start_parallel = MoveParallel.MoveParallel(name="BaseStartParallel")
        base_start_parallel.add_children(
            [
                # Return left arm joints to the base preset.
                MoveJoint.MOVEJ(
                    name=f"{left_robot}_BaseStart",
                    action_client=action_clients[left_robot],
                    action_goal=base_start_left_joint_goal,
                    robot_name=left_robot,
                    timeout=MOVE_TIME,
                ),
                # Return right arm joints to the base preset.
                MoveJoint.MOVEJ(
                    name=f"{right_robot}_BaseStart",
                    action_client=action_clients[right_robot],
                    action_goal=base_start_right_joint_goal,
                    robot_name=right_robot,
                    timeout=MOVE_TIME,
                ),
                # Close left gripper while returning to base.
                Gripper.GOTO(
                    name=f"{left_robot}_BaseStartClose",
                    action_client=action_clients[left_robot],
                    action_goal=left_robot_blackboard.gripper_close_pos,
                    force=left_robot_blackboard.gripper_close_force,
                    timeout=GRIPPER_TIME,
                    robot_name=left_robot,
                ),
                # Close right gripper while returning to base.
                Gripper.GOTO(
                    name=f"{right_robot}_BaseStartClose",
                    action_client=action_clients[right_robot],
                    action_goal=right_robot_blackboard.gripper_close_pos,
                    force=right_robot_blackboard.gripper_close_force,
                    timeout=GRIPPER_TIME,
                    robot_name=right_robot,
                ),
            ]
        )

        # Execute grasp sequence first, then finish with dual-arm policy execution.
        root = py_trees.composites.Sequence(name="DualGrasp", memory=True)
        # root.add_children([s_init1, pose_estimator, move_holding, move_approach, move_horizontal])
        root.add_children(
            [
                pose_estimator, 
                move_approach_seq,
                # move_horizontal_wp, 
                above_mold_start_parallel,
                unfreeze_fingers,
                # dual_policy_seq, # for pick-only dry run...
                base_start_parallel])

        return root
