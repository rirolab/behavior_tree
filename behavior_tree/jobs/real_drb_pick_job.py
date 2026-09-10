import json

import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import (
    Gripper,
    JointStateLogger,
    MoveJoint,
    MoveParallel,
    MovePose,
    Policy,
    PolicyServer,
    RealControllerCommand,
    RingWorldModel,
    Trigger,
    Wait,
)
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Dual-arm real-hardware pick job that replays configured joint pose presets.
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
        if step.get("primitive_action") != "real_drb_pick":
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
                "real_drb_pick_job: rejecting new goal, previous still in the pipeline"
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

    def make_move_pose_with_logger(
        self,
        name,
        action_client,
        action_goal,
        robot_name,
        timeout,
        joint_logger_kwargs,
        enable_joint_logging=True,
    ):
        """
        Chain one Cartesian move and optionally one joint-state logger in one sequence.
        """
        # Keep the wrapper node stable even when logging is disabled.
        move_pose_with_logger = py_trees.composites.Sequence(
            name=f"{name}WithLogger",
            memory=True,
        )

        # Build the Cartesian replay motion that is required for task execution.
        move_pose = MovePose.MOVEP(
            name=name,
            action_client=action_client,
            action_goal=action_goal,
            robot_name=robot_name,
            timeout=timeout,
        )
        if not enable_joint_logging:
            move_pose_with_logger.add_child(move_pose)
            return move_pose_with_logger

        # Attach the optional joint-state dump after the motion succeeds.
        joint_logger = JointStateLogger.JOINT_STATE_LOGGING(
            name=f"{name}JointLogger",
            **joint_logger_kwargs,
        )

        move_pose_with_logger.add_children([move_pose, joint_logger])
        return move_pose_with_logger

    def make_policy_goal(self, step, robot_name, step_idx=None, default_timeout=5.0):
        """
        Build one real HIL-SERL policy goal for the requested arm.
        """
        # Merge the shared pick step with the requested robot policy block.
        policy_goal = self.make_robot_specific_goal(step, robot_name, step_idx)
        if policy_goal is None:
            return None
        if not bool(policy_goal.get("skill_id")):
            return None

        # Route the merged payload through the arm-client policy execution path.
        policy_goal["primitive_action"] = "policy_execute"
        try:
            policy_goal["timeout"] = float(policy_goal.get("timeout", default_timeout))
        except (TypeError, ValueError):
            return None
        return policy_goal

    def make_policy_preload_requests(self, step, step_idx):
        """
        Build the left-arm policy preload request used by policy-mode pick.
        """
        # Ignore unrelated steps and non-policy pick modes.
        if not self.acceptable_step(step):
            return []
        global_blackboard = py_trees.blackboard.Client()
        global_blackboard.register_key(
            key="drb_mode",
            access=py_trees.common.Access.READ,
        )
        if global_blackboard.drb_mode != "policy":
            return []

        # Preload only the left policy that this job executes in policy mode.
        left_robot, _ = self.resolve_left_right_robots(step)
        if left_robot is None:
            return []
        policy_goal = self.make_policy_goal(step, left_robot, step_idx)
        if policy_goal is None:
            return []
        return [(left_robot, policy_goal)]

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
            raise RuntimeError("real_drb_pick_job: robot_names must be provided")
        if not self.acceptable_step(goal[idx]):
            return None

        # Keep one shared timeout for every joint replay stage.
        plan_name = "Plan" + idx
        step = goal[idx]
        action_clients = action_client
        # MOVE_TIME = 5.0
        MOVE_TIME = 2.0
        GRIPPER_MOVE_TIME = 0.5
        # GRIPPER_MOVE_TIME = 1.0

        # Resolve the grounded left/right robot names and verify both clients exist.
        left_robot, right_robot = self.resolve_left_right_robots(step)
        if left_robot is None or right_robot is None or left_robot == right_robot:
            raise RuntimeError(
                "real_drb_pick_job: expected one left robot and one right robot"
            )
        if left_robot not in action_clients or right_robot not in action_clients:
            raise RuntimeError(
                "real_drb_pick_job: missing action client for one or more robots"
            )

        # Read whether pose replay joint-state logging should be part of the BT path.
        enable_pose_replay_joint_logging = False
        if self._node.has_parameter("enable_pose_replay_joint_logging"):
            enable_pose_replay_joint_logging = self._node.get_parameter(
                "enable_pose_replay_joint_logging"
            ).value
            if not isinstance(enable_pose_replay_joint_logging, bool):
                raise RuntimeError(
                    "real_drb_pick_job: enable_pose_replay_joint_logging must be a bool"
                )

        # Reuse one shared joint-state logger configuration for pose replay logging.
        joint_logger_kwargs = {
            "warmup_sec": 1.0,
            "joint_states_topic": "/joint_states",
            "output_dir": "/tmp/behavior_tree_joint_logs",
            "left_robot_name": left_robot,
            "right_robot_name": right_robot,
        }

        # Read shared pose presets from the global blackboard.
        global_blackboard = py_trees.blackboard.Client()
        global_blackboard.register_key(
            key="pose_presets",
            access=py_trees.common.Access.READ,
        )
        global_blackboard.register_key(
            key="drb_mode",
            access=py_trees.common.Access.READ,
        )
        ############## For test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        does_policy_grasp = global_blackboard.drb_mode == "policy"
        does_manual_grasp = global_blackboard.drb_mode == "manual"
        # does_policy_grasp = False
        # does_manual_grasp = True
        

        # External pick actor owns reward checking during policy-mode execution.
        # reward_check_trigger_topic = ""
        # if does_policy_grasp:
        #     if self._node.has_parameter("reward_check_trigger_topic"):
        #         reward_check_trigger_topic = str(
        #             self._node.get_parameter("reward_check_trigger_topic").value
        #         ).strip()
        #     if not reward_check_trigger_topic:
        #         raise RuntimeError(
        #             "real_drb_pick_job: reward_check_trigger_topic is required in policy mode"
        #         )

        # Resolve the initial dual-arm joint preset.
        base_start = global_blackboard.pose_presets.get("base_start")
        if base_start is None:
            console.logerror("RealDrbPick: Missing pose preset [base_start]")
            return None
        base_start_left_joint_goal = base_start.get("left_joint_pos")
        base_start_right_joint_goal = base_start.get("right_joint_pos")
        if base_start_left_joint_goal is None or base_start_right_joint_goal is None:
            console.logerror(
                "RealDrbPick: Missing left/right joint preset in pose preset [base_start]"
            )
            return None

        # Resolve the second dual-arm preset used before the regrasp approach.
        stack_side_start = global_blackboard.pose_presets.get("stack_side_start2")
        if stack_side_start is None:
            console.logerror("RealDrbPick: Missing pose preset [stack_side_start2]")
            return None
        stack_side_start_left_joint_goal = stack_side_start.get("left_joint_pos")
        stack_side_start_right_joint_goal = stack_side_start.get("right_joint_pos")
        if (
            stack_side_start_left_joint_goal is None
            or stack_side_start_right_joint_goal is None
        ):
            console.logerror(
                "RealDrbPick: Missing left/right joint preset in pose preset [stack_side_start2]"
            )
            return None

        # Resolve the stack-side left gripper opening for policy-mode staging.
        if does_policy_grasp:
            stack_side_start_left_gripper_values = stack_side_start.get(
                "left_gripper_values"
            )
            if stack_side_start_left_gripper_values is None:
                console.logerror(
                    "RealDrbPick: Missing left_gripper_values in pose preset [stack_side_start2]"
                )
                return None
            stack_side_start_left_gripper_open = (
                stack_side_start_left_gripper_values.get("open")
            )
            if stack_side_start_left_gripper_open is None:
                console.logerror(
                    "RealDrbPick: Missing left gripper open preset in pose preset [stack_side_start2]"
                )
                return None

        # Read left-arm gripper parameters for active pick-mode gripper commands.
        if does_policy_grasp or does_manual_grasp:
            left_blackboard = py_trees.blackboard.Client(namespace=left_robot)
            left_blackboard.register_key(
                key="gripper_open_pos",
                access=py_trees.common.Access.READ,
            )
            left_blackboard.register_key(
                key="gripper_close_pos",
                access=py_trees.common.Access.READ,
            )
            left_blackboard.register_key(
                key="gripper_open_force",
                access=py_trees.common.Access.READ,
            )
            left_blackboard.register_key(
                key="gripper_close_force",
                access=py_trees.common.Access.READ,
            )

        # External pick actor replaces the arm-client policy payload in policy mode.
        if does_policy_grasp:
            # Resolve the long-lived policy server endpoint used by BT policy triggers.
            policy_server_url = "http://127.0.0.1:5080"
            if self._node.has_parameter("policy_server_url"):
                policy_server_url = str(
                    self._node.get_parameter("policy_server_url").value
                ).strip() or policy_server_url
            pick_policy_timeout = 60.0
            if self._node.has_parameter("policy_server_pick_timeout_sec"):
                pick_policy_timeout = float(
                    self._node.get_parameter("policy_server_pick_timeout_sec").value
                )
            camera_prepare_timeout = 140.0
            if self._node.has_parameter("policy_server_camera_prepare_timeout_sec"):
                camera_prepare_timeout = float(
                    self._node.get_parameter("policy_server_camera_prepare_timeout_sec").value
                )
            pick_camera_prepare = PolicyServer.PREPARE_POLICY(
                name="PreparePickPolicyCamera",
                policy="pick",
                server_url=policy_server_url,
                timeout=camera_prepare_timeout,
            )

        # Move both robots to the initial joint preset in parallel.
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

        # Estimate the ring target frame that the right arm will approach.
        pose_estimator = RingWorldModel.POSE_ESTIMATOR(
            name=plan_name,
            object_dict={"target": "dual_grasp_target"},
            robot_names=robot_names,
            holding_robot=left_robot,
            approach_robot=right_robot,
            tf_buffer=kwargs["tf_buffer"],
        )

        # Assemble the full pick sequence in execution order.
        root = py_trees.composites.Sequence(name="RealDrbPick", memory=True)

        if does_policy_grasp:
            # Replay the measured left-arm joint start while moving the right
            # arm to its existing Cartesian regrasp target.
            stack_side_start_parallel = MoveParallel.MoveParallel(
                name="StackSideStartParallel"
            )
            stack_side_start_parallel.add_children(
                [
                    MoveJoint.MOVEJ(
                        name=f"{left_robot}_StackSideStart",
                        action_client=action_clients[left_robot],
                        action_goal=stack_side_start_left_joint_goal,
                        robot_name=left_robot,
                        timeout=3.0,
                    ),
                    self.make_move_pose_with_logger(
                        name=f"{right_robot}_StackSideStart",
                        action_client=action_clients[right_robot],
                        action_goal={"pose": plan_name + "/regrasp_target_down_right"},
                        robot_name=right_robot,
                        timeout=MOVE_TIME,
                        joint_logger_kwargs=joint_logger_kwargs,
                        enable_joint_logging=enable_pose_replay_joint_logging,
                    ),
                    Gripper.GOTO(
                        name="LeftPolicyGripperOpen",
                        action_client=action_clients[left_robot],
                        action_goal=stack_side_start_left_gripper_open,
                        force=left_blackboard.gripper_open_force,
                        timeout=GRIPPER_MOVE_TIME,
                        robot_name=left_robot,
                    )
                ]
            )

            # Prewarm RGB/LAS and run an early SAM2 tracking pass while the
            # complete move to the pick preparation pose is in flight. The
            # policy server reseeds SAM2 again after this parallel joins so
            # policy step 1 uses tracking anchored at the final start pose.
            pick_preparation_motion = py_trees.composites.Sequence(
                name="PickPreparationMotion",
                memory=True,
            )
            pick_preparation_motion.add_children(
                [
                    base_start_parallel,
                    pose_estimator,
                    stack_side_start_parallel,
                ]
            )
            pick_preparation_with_camera = MoveParallel.MoveParallel(
                name="PickPreparationWithCameraPrewarm"
            )
            pick_preparation_with_camera.add_children(
                [
                    pick_preparation_motion,
                    pick_camera_prepare,
                ]
            )

            # Switch the left robot into cartesian impedance before policy execution.
            left_policy_switch_cartesian = RealControllerCommand.REAL_CONTROLLER_COMMAND(
                name="SwitchLeftCartesianBeforePickPolicy",
                command={
                    "action_type": "switchController",
                    "controller_profile": "cartesian_impedance_controller",
                    "target_arms": left_robot,
                },
                timeout=10.0,
            )

            wait_until_trigger_temp = Wait.WAIT_UNTIL_TRIGGER(
                name="WaitUntilTriggerTemp",
                robot_name=left_robot,
            )

            # Open the cartesian command HTTP gate before the external actor runs.
            pick_cartesian_command_gate_enable = (
                RealControllerCommand.CARTESIAN_COMMAND_HTTP_GATE(
                    name="EnablePickCartesianCommandHttpGate",
                    command={
                        "reset_mode": "pick_reset",
                        "cartesian_command_gate": "enable",
                    },
                    timeout=10.0,
                )
            )

            # Trigger the preloaded pick policy through the long-lived HIL-SERL server.
            pick_policy_request = PolicyServer.RUN_POLICY(
                name="RunPickPolicyServer",
                policy="pick",
                server_url=policy_server_url,
                timeout=pick_policy_timeout,
            )

            # Close the cartesian command HTTP gate after the external actor finishes.
            pick_cartesian_command_gate_pause = (
                RealControllerCommand.CARTESIAN_COMMAND_HTTP_GATE(
                    name="PausePickCartesianCommandHttpGate",
                    command={
                        "cartesian_command_gate": "pause",
                    },
                    timeout=10.0,
                )
            )

            # Execute the left-arm policy through complex_action_client.
            # left_run_policy = Policy.MOVEBYPOLICY(
            #     name=f"{left_robot}_MoveByPolicy",
            #     action_client=action_clients[left_robot],
            #     action_goal=left_robot_policy_goal,
            #     timeout=float(left_robot_policy_goal.get("timeout", MOVE_TIME)),
            #     robot_name=left_robot,
            # )

            # Enable reward checking only while the left policy is running.
            # left_run_policy_with_reward_trigger = Trigger.RUN_WITH_BOOL_TRIGGER(
            #     name="LeftPickPolicyRewardTrigger",
            #     child=left_run_policy,
            #     topic_name=reward_check_trigger_topic,
            #     start_value=True,
            #     stop_value=False,
            # )

            # Return the left robot to JTC before the final gripper close.
            left_policy_switch_jtc = RealControllerCommand.REAL_CONTROLLER_COMMAND(
                name="SwitchLeftJtcAfterPickPolicy",
                command={
                    "action_type": "switchController",
                    "controller_profile": "joint_trajectory_controller",
                    "target_arms": left_robot,
                },
                timeout=10.0,
            )

            # Always pause the command gate and restore JTC, including camera/policy failures.
            pick_policy_body = py_trees.composites.Sequence(
                name="PickPolicyControlBody",
                memory=True,
            )
            pick_policy_body.add_children(
                [
                    pick_cartesian_command_gate_enable,
                    pick_policy_request,
                ]
            )
            left_external_pick_policy_seq = PolicyServer.RUN_WITH_CLEANUP(
                name="LeftExternalPickPolicyWithCleanup",
                body=pick_policy_body,
                cleanup_children=[
                    pick_cartesian_command_gate_pause,
                    left_policy_switch_jtc,
                ],
            )

            # Close the left gripper after the policy handoff returns to JTC.
            left_policy_gripper_close = Gripper.GOTO(
                name="LeftPolicyGripperClose",
                action_client=action_clients[left_robot],
                action_goal=left_blackboard.gripper_close_pos,
                force=left_blackboard.gripper_close_force,
                timeout=GRIPPER_MOVE_TIME,
                robot_name=left_robot,
            )

            root.add_children(
                [
                    pick_preparation_with_camera,
                    left_policy_switch_cartesian,
                    # wait_until_trigger_temp,
                    left_external_pick_policy_seq,
                    left_policy_gripper_close,
                ]
            )

        if does_manual_grasp:
            # Replay the measured left-arm joint start and the existing right-arm
            # Cartesian regrasp target before manual grasping.
            stack_side_start_parallel = MoveParallel.MoveParallel(
                name="StackSideStartParallel"
            )
            stack_side_start_parallel.add_children(
                [
                    MoveJoint.MOVEJ(
                        name=f"{left_robot}_StackSideStart",
                        action_client=action_clients[left_robot],
                        action_goal=stack_side_start_left_joint_goal,
                        robot_name=left_robot,
                        timeout=MOVE_TIME,
                    ),
                    self.make_move_pose_with_logger(
                        name=f"{right_robot}_StackSideStart",
                        action_client=action_clients[right_robot],
                        action_goal={"pose": plan_name + "/regrasp_target_down_right"},
                        robot_name=right_robot,
                        timeout=MOVE_TIME,
                        joint_logger_kwargs=joint_logger_kwargs,
                        enable_joint_logging=enable_pose_replay_joint_logging,
                    ),
                ]
            )

            # Append the manual grasp steps directly after the replay stages finish.
            left_manual_grasp_move = MoveJoint.MOVEJ(
                name=f"{left_robot}_ManualGraspBaseStart",
                action_client=action_clients[left_robot],
                action_goal=base_start_left_joint_goal,
                robot_name=left_robot,
                timeout=MOVE_TIME,
            )

            # Open the left gripper and hand control over to the operator.
            left_manual_grasp_open = Gripper.GOTO(
                name="LeftManualGraspOpen",
                action_client=action_clients[left_robot],
                action_goal=left_blackboard.gripper_open_pos,
                force=left_blackboard.gripper_open_force,
                timeout=GRIPPER_MOVE_TIME,
                robot_name=left_robot,
            )

            # Wait until an external trigger confirms the operator is ready to grasp.
            left_manual_grasp_wait = Wait.WAIT_UNTIL_TRIGGER(
                name="LeftManualGraspWaitUntilTrigger",
                robot_name=left_robot,
            )

            # Close the left gripper once the operator-triggered grasp should begin.
            left_manual_grasp_close = Gripper.GOTO(
                name="LeftManualGraspClose",
                action_client=action_clients[left_robot],
                action_goal=left_blackboard.gripper_close_pos,
                force=left_blackboard.gripper_close_force,
                timeout=GRIPPER_MOVE_TIME,
                robot_name=left_robot,
            )

            root.add_children(
                [
                    base_start_parallel,
                    pose_estimator,
                    stack_side_start_parallel,
                    left_manual_grasp_move,
                    left_manual_grasp_open,
                    left_manual_grasp_wait,
                    left_manual_grasp_close,
                ]
            )

        if not does_policy_grasp and not does_manual_grasp:
            root.add_children([base_start_parallel, pose_estimator])

        return root
