import json
import sys

import numpy as np
import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import IsaacSceneCommand, MoveJoint, MovePose, MoveParallel, Policy, Wait, RingWorldModel, Gripper, WorldModel
from behavior_tree.subtrees import JointStateLogger
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


##############################################################################
# Behaviours
##############################################################################


class Move(base_job.BaseJob):
    """
    A job handler that replays dual-arm joint presets as parallel motion pairs.
    """

    def __init__(self, node):
        """
        Tune into a channel for incoming goal requests. This is a simple
        subscriber here but more typically would be a service or action interface.
        """
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
        # Accept only dedicated dual-arm drb_pick steps.
        if step.get("primitive_action") != "drb_pick":
            return False
        elif not self.check_robot_count(step, num_robot_required=2):
            return False
        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable pick step is well-formed enough to keep
        the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        # Accept only steps owned by this job.
        if not self.acceptable_step(step):
            return StepValidationResult.NOT_APPLICABLE

        # Require exactly one left robot and one right robot.
        grounded_robot_names = make_string_list(step.get("robot", []))
        left_robot = next((robot_name for robot_name in grounded_robot_names if "left" in robot_name), None)
        right_robot = next((robot_name for robot_name in grounded_robot_names if "right" in robot_name), None)
        bt_robot_names = getattr(self._node, "robot_names", [])
        if left_robot is None or right_robot is None or left_robot == right_robot:
            return StepValidationResult.REJECT_GOAL
        if bt_robot_names and (
            left_robot not in bt_robot_names or right_robot not in bt_robot_names
        ):
            return StepValidationResult.REJECT_GOAL

        # Validate any robot-specific policy payloads attached to this step.
        for robot_name in [left_robot, right_robot]:
            robot_goal = self.make_robot_specific_goal(step, robot_name, step.get("step_idx"))
            if robot_goal is None:
                return StepValidationResult.REJECT_GOAL
            if robot_goal.get("implementation") == "policy" and (
                not bool(robot_goal.get("skill_id"))
                or not bool(robot_goal.get("scenario_name"))
            ):
                return StepValidationResult.REJECT_GOAL

        return StepValidationResult.ACCEPT_GOAL

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        # Cache the full grounding when one step belongs to this job.
        if self.goal:
            self._node.get_logger().error("pick_job: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)["params"]
            for i in range(len(grounding.keys())):
                step = grounding.get(str(i + 1))
                if step is None:
                    continue
                if self.acceptable_step(step):
                    self.goal = grounding
                    break

    def add_last_joint_offset(self, joint_goal, last_joint_offset):
        """
        Return one joint goal with the last joint offset applied.
        """
        # Copy the goal before shifting the last joint target.
        adjusted_joint_goal = list(joint_goal)
        adjusted_joint_goal[-1] += last_joint_offset
        return adjusted_joint_goal

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
        # Require a dual-arm goal owned by this job.
        if robot_names is None:
            raise RuntimeError("drb_pick_job: robot_names must be provided")
        if not self.acceptable_step(goal[idx]):
            return None

        # Keep one shared timeout for every joint replay stage.
        plan_name = "Plan" + idx
        step = goal[idx]
        action_clients = action_client
        MOVE_TIME = 0.25
        GRIPPER_TIME = 0.25

        # Resolve the grounded left/right robot names for parallel moves.
        grounded_robot_names = make_string_list(step.get("robot", []))
        left_robot = next((robot_name for robot_name in grounded_robot_names if "left" in robot_name), None)
        right_robot = next((robot_name for robot_name in grounded_robot_names if "right" in robot_name), None)
        if left_robot is None or right_robot is None or left_robot == right_robot:
            raise RuntimeError("drb_pick_job: expected one left robot and one right robot")

        # Resolve any robot-specific policy payloads attached to this step.
        left_robot_policy_goal = self.make_robot_specific_goal(step, left_robot, idx)
        if left_robot_policy_goal is None:
            raise RuntimeError("drb_pick_job: failed to resolve left robot goal")

        # Enable left-policy recovery only when the left-arm policy payload requests it.
        recovery_policy_enabled = bool(left_robot_policy_goal.get("recovery_policy", False))

        # Keep one shared joint logger configuration for every pose move.
        joint_logger_kwargs = {
            "warmup_sec": 1.0,
            "joint_states_topic": "/joint_states",
            "output_dir": "/tmp/behavior_tree_joint_logs",
            "left_robot_name": left_robot,
            "right_robot_name": right_robot,
        }

        # Read robot-local init joints and shared preset catalog from blackboard.
        left_blackboard = py_trees.blackboard.Client(namespace=left_robot)
        left_blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)
        right_blackboard = py_trees.blackboard.Client(namespace=right_robot)
        right_blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)
        global_blackboard = py_trees.blackboard.Client()
        global_blackboard.register_key(key="pose_presets", access=py_trees.common.Access.READ)
        global_blackboard.register_key(key="drb_mode", access=py_trees.common.Access.READ)
        does_teleport_ring = global_blackboard.drb_mode == "teleport_ring"
        
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

        # Queue base_start as one left/right parallel stage.
        base_start_parallel = MoveParallel.MoveParallel(name="BaseStartParallel")
        base_start_parallel.add_children(
            [
                MoveJoint.MOVEJ(
                    name=f"{left_robot}_BaseStart",
                    action_client=action_clients[left_robot],
                    action_goal=base_start_left_joint_goal,
                    robot_name=left_robot,
                    timeout=MOVE_TIME,
                ),
                MoveJoint.MOVEJ(
                    name=f"{right_robot}_BaseStart",
                    action_client=action_clients[right_robot],
                    action_goal=base_start_right_joint_goal,
                    robot_name=right_robot,
                    timeout=MOVE_TIME,
                ),
            ]
        )

        # Resolve stack_side_start joint targets.
        stack_side_start = global_blackboard.pose_presets.get("stack_side_start3")
        if stack_side_start is None:
            console.logerror("Pick: Missing pose preset [stack_side_start3]")
            return None
        stack_side_start_left_joint_goal = stack_side_start.get("left_joint_pos")
        stack_side_start_right_joint_goal = stack_side_start.get("right_joint_pos")
        if stack_side_start_left_joint_goal is None or stack_side_start_right_joint_goal is None:
            console.logerror("Pick: Missing left/right joint preset in pose preset [stack_side_start3]")
            return None

        pose_estimator = RingWorldModel.POSE_ESTIMATOR(
            name=plan_name,
            object_dict={'target': 'dual_grasp_target'},
            robot_names=robot_names,
            holding_robot=left_robot,
            approach_robot=right_robot,
            tf_buffer=kwargs["tf_buffer"],
        )

        # Queue stack_side_start as one left/right parallel stage.
        stack_side_start_parallel = MoveParallel.MoveParallel(name="StackSideStartParallel")
        stack_side_start_parallel.add_children(
            [
                # Replay left-arm joint preset in parallel branch.
                MoveJoint.MOVEJ(
                    name=f"{left_robot}_StackSideStart",
                    action_client=action_clients[left_robot],
                    action_goal=stack_side_start_left_joint_goal,
                    robot_name=left_robot,
                    timeout=MOVE_TIME,
                ),
                # Run right-arm pose move and log joints after completion.
                self.make_move_pose_with_logger(
                    name=f"{right_robot}_StackSideStart",
                    action_client=action_clients[right_robot],
                    action_goal={"pose": plan_name + "/regrasp_target_down_right"},
                    robot_name=right_robot,
                    timeout=MOVE_TIME,
                    joint_logger_kwargs=joint_logger_kwargs,
                )
            ]
        )

        # Append left-arm policy execution and return-to-stack sequence when requested.
        left_policy_return_seq = py_trees.composites.Sequence(
            name="LeftPolicyReturnSeq",
            memory=True,
        )
        left_switch_reward_profile = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="LeftSwitchRewardProfile",
            command={
                "action_type": "setRewardProfile",
                "reward_profile": step[left_robot].get("scenario_name"),
            },
            timeout=10.0,
        )
        left_switch_controller_in = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="LeftSwitchControllerIn",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "cartesian_motion_controller",
                "target_arms": left_robot,
            },
            timeout=10.0,
        )
        left_run_policy = Policy.MOVEBYPOLICY(
            name=f"{left_robot}_MoveByPolicy",
            action_client=action_clients[left_robot],
            action_goal=left_robot_policy_goal,
            timeout=float(left_robot_policy_goal.get("timeout", 30.0)),
            robot_name=left_robot,
        )
        left_switch_controller_out = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="LeftSwitchControllerOut",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "joint_trajectory_controller",
                "target_arms": left_robot,
            },
            timeout=10.0,
        )
        wait_after_left_switch_controller_out = Wait.WAIT(
            name="WaitAfterReturn",
            duration=1.0,
            robot_name=left_robot,
        )
        scene_cmd_temp = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name=f"{left_robot}_FreezeFingerJoint",
            command={
                "action_type": "freezeFingerJoint",
                "arm": left_robot,
                "enabled": True
            },
            timeout=5.0,
        )
        left_policy_return_seq.add_children(
            [
                left_switch_reward_profile,
                left_switch_controller_in,
                left_run_policy,
                left_switch_controller_out,
                wait_after_left_switch_controller_out,
                scene_cmd_temp
            ]
        )
        left_policy_branch = left_policy_return_seq
        if recovery_policy_enabled:
            # Build the recovery branch that switches out and moves only the left arm.
            left_recovery_switch_controller_out = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
                name="LeftRecoverySwitchControllerOut",
                command={
                    "action_type": "setRobotDriveGainProfileAndSwitchController",
                    "robot_drive_gain_profile": "joint_trajectory_controller",
                    "target_arms": left_robot,
                },
                timeout=10.0,
            )
            left_recovery_stack_side_start = MoveJoint.MOVEJ(
                name=f"{left_robot}_RecoveryStackSideStart",
                action_client=action_clients[left_robot],
                action_goal=stack_side_start_left_joint_goal,
                robot_name=left_robot,
                timeout=4*MOVE_TIME,
            )
            left_policy_recovery_seq = py_trees.composites.Sequence(
                name="LeftPolicyRecoverySeq",
                memory=True,
            )
            left_policy_recovery_seq.add_children(
                [
                    left_recovery_switch_controller_out,
                    left_recovery_stack_side_start,
                ]
            )

            # Convert successful recovery into a retry-triggering failure for the selector.
            left_policy_recovery_selector = py_trees.composites.Selector(
                name="LeftPolicyAttemptOrRecovery",
                memory=True,
            )
            left_policy_recovery_selector.add_children(
                [
                    left_policy_return_seq,
                    py_trees.decorators.SuccessIsFailure(
                        name="RepeatAfterLeftPolicyRecovery",
                        child=left_policy_recovery_seq,
                    ),
                ]
            )
            left_policy_branch = py_trees.decorators.Retry(
                name="LeftPolicyRecoveryRetryForever",
                child=left_policy_recovery_selector,
                num_failures=sys.maxsize,
            )
        left_base_start_again = MoveJoint.MOVEJ(
                    name=f"{left_robot}_BaseStartAgain",
                    action_client=action_clients[left_robot],
                    action_goal=base_start_left_joint_goal,
                    robot_name=left_robot,
                    timeout=MOVE_TIME,
        )

        root = py_trees.composites.Sequence(name="Pick", memory=True)
        root.add_children(
            [
                base_start_parallel, 
                pose_estimator, 
                stack_side_start_parallel, 
            ]
        )

        # Policy grasp ring
        if not does_teleport_ring:
            root.add_children(
                [
                    left_policy_branch,
                    # left_base_start_again
                ]
            )

        # Teleport ring (not working for now)
        if does_teleport_ring:

            # Get parameters
            left_blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
            left_blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
            left_blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
            left_blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)

            pose_est1 = WorldModel.POSE_ESTIMATOR(
                name="Plan" + idx,
                object_dict={'target': step[left_robot]['object']},
                tf_buffer=kwargs['tf_buffer'],
                robot_name=left_robot,
            )
            # Run return move and gripper open in parallel for ring grasp prep.
            left_ring_grasp_parallel = MoveParallel.MoveParallel(name="LeftRingGraspParallel")
            # Return left arm to pick pose and log joints after completion.
            left_return_pick_init = self.make_move_pose_with_logger(
                name="LeftReturnPickInit",
                action_client=action_clients[left_robot],
                action_goal={'pose': "Plan" + idx + "/grasp_top_pose"},
                robot_name=left_robot,
                timeout=MOVE_TIME,
                joint_logger_kwargs=joint_logger_kwargs,
            )
            # Open left gripper while the return branch is running.
            left_open_gripper = Gripper.GOTO(
                name="LeftOpenGripper",
                action_client=action_clients[left_robot],
                action_goal=left_blackboard.gripper_open_pos,
                force=left_blackboard.gripper_open_force,
                timeout=GRIPPER_TIME,
                robot_name=left_robot
            )
            left_ring_grasp_parallel.add_children([left_return_pick_init, left_open_gripper])

            # Teleport ring and grasp
            scene_cmd1 = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
                name="TeleportActiveRingRigidToPickPose",
                command={
                    "action_type": "teleportActiveRingRigid",
                    "target_frame": "ring_stack_anchor_grasp_top",
                    "offset_xyz": [0.0, 0.01, 0.0],
                    "local_axis": "x",
                    "angle_deg": 90.0,
                },
                timeout=5.0,
            )
            left_close_gripper = Gripper.GOTO(
                name="Close",
                action_client=action_clients[left_robot],
                action_goal=left_blackboard.gripper_close_pos,
                force=left_blackboard.gripper_close_force,
                timeout=GRIPPER_TIME,
                robot_name=left_robot
            )
            scene_cmd2 = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
                name="EnableActiveRingGravityAndDeformable",
                command={
                    "action_type": "enableActiveRingGravityAndDeformable",
                },
                timeout=5.0,
            )
            scene_cmd3 = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
                name=f"{left_robot}_FreezeFingerJoint",
                command={
                    "action_type": "freezeFingerJoint",
                    "arm": left_robot,
                    "enabled": True
                },
                timeout=5.0,
            )

            root.add_children(
                [
                    pose_est1,
                    left_ring_grasp_parallel,
                    scene_cmd1,
                    left_close_gripper,
                    scene_cmd2,
                    scene_cmd3
                ]
            )

        return root
