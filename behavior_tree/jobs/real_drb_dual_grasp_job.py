import json

import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import (
    Gripper,
    IsaacSceneCommand,
    JointStateLogger,
    MoveJoint,
    MoveParallel,
    MovePose,
    Policy,
    RealControllerCommand,
    RingWorldModel,
    Trigger,
    Wait,
)
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Dual-arm real-hardware dual grasp job that moves the holding and approach
    robots toward the ring regrasp target poses.
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
        if step.get("primitive_action") != "real_drb_dual_grasp":
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
        holding_robot = step.get("holding_robot")
        approach_robot = step.get("approach_robot")
        bt_robot_names = getattr(self._node, "robot_names", [])

        if len(grounded_robot_names) != 2 or len(set(grounded_robot_names)) != 2:
            return StepValidationResult.REJECT_GOAL
        if left_robot is None or right_robot is None or left_robot == right_robot:
            return StepValidationResult.REJECT_GOAL
        if not holding_robot or not approach_robot or holding_robot == approach_robot:
            return StepValidationResult.REJECT_GOAL
        if holding_robot not in grounded_robot_names or approach_robot not in grounded_robot_names:
            return StepValidationResult.REJECT_GOAL
        if bt_robot_names and (
            left_robot not in bt_robot_names
            or right_robot not in bt_robot_names
            or holding_robot not in bt_robot_names
            or approach_robot not in bt_robot_names
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
                "real_drb_dual_grasp_job: rejecting new goal, previous still in the pipeline"
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
    ):
        """
        Chain one Cartesian move and one joint-state logger in one sequence.
        """
        move_pose_with_logger = py_trees.composites.Sequence(
            name=f"{name}WithLogger",
            memory=True,
        )
        move_pose = MovePose.MOVEP(
            name=name,
            action_client=action_client,
            action_goal=action_goal,
            robot_name=robot_name,
            timeout=timeout,
        )
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
        # Merge shared dual-grasp fields with the requested robot policy block.
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
        Build both arm policy preload requests used by policy-mode dual grasp.
        """
        # Ignore unrelated steps and non-policy dual-grasp modes.
        if not self.acceptable_step(step):
            return []
        global_blackboard = py_trees.blackboard.Client()
        global_blackboard.register_key(
            key="drb_mode",
            access=py_trees.common.Access.READ,
        )
        if global_blackboard.drb_mode != "policy":
            return []

        # Preload left and right policies before controller switching starts.
        requests = []
        left_robot, right_robot = self.resolve_left_right_robots(step)
        for robot_name in (left_robot, right_robot):
            if robot_name is None:
                return []
            policy_goal = self.make_policy_goal(step, robot_name, step_idx)
            if policy_goal is None:
                return []
            requests.append((robot_name, policy_goal))
        return requests

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
            raise RuntimeError("real_drb_dual_grasp_job: robot_names must be provided")

        if not self.acceptable_step(goal[idx]):
            return None

        # Read the current command and normalize the grounded robot roles.
        step = goal[idx]
        grounded_robots = make_string_list(step.get("robot", []))
        holding_robot = step.get("holding_robot")
        approach_robot = step.get("approach_robot")
        action_clients = action_client
        plan_name = "Plan" + idx
        MOVE_TIME = 5.0
        GRIPPER_TIME = 1.0

        # Resolve the grounded left/right robot names and validate both roles.
        left_robot, right_robot = self.resolve_left_right_robots(step)
        if left_robot is None or right_robot is None or left_robot == right_robot:
            raise RuntimeError(
                "real_drb_dual_grasp_job: expected one left robot and one right robot"
            )
        if holding_robot not in grounded_robots or approach_robot not in grounded_robots:
            raise RuntimeError(
                "real_drb_dual_grasp_job: expected holding and approach robots in the grounding"
            )
        if holding_robot == approach_robot:
            raise RuntimeError(
                "real_drb_dual_grasp_job: holding and approach robots must differ"
            )
        if left_robot not in action_clients or right_robot not in action_clients:
            raise RuntimeError(
                "real_drb_dual_grasp_job: missing action client for one or more robots"
            )

        # Reuse one shared joint-state logger configuration for pose replay logging.
        joint_logger_kwargs = {
            "warmup_sec": 1.0,
            "joint_states_topic": "/joint_states",
            "output_dir": "/tmp/behavior_tree_joint_logs",
            "left_robot_name": left_robot,
            "right_robot_name": right_robot,
        }

        # Read shared pose presets and robot-local gripper parameters from blackboard.
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

        # Get drb_mode
        global_blackboard.register_key(
            key="drb_mode",
            access=py_trees.common.Access.READ,
        )
        does_policy_grasp = global_blackboard.drb_mode == "policy"
        does_manual_grasp = global_blackboard.drb_mode == "manual"

        # Read stage-specific reward trigger topics for policy-mode placement and fit.
        stage1_reward_check_trigger_topic = ""
        stage2_reward_check_trigger_topic = ""
        if does_policy_grasp:
            if self._node.has_parameter("stage1_reward_check_trigger_topic"):
                stage1_reward_check_trigger_topic = str(
                    self._node.get_parameter("stage1_reward_check_trigger_topic").value
                ).strip()
            if self._node.has_parameter("stage2_reward_check_trigger_topic"):
                stage2_reward_check_trigger_topic = str(
                    self._node.get_parameter("stage2_reward_check_trigger_topic").value
                ).strip()
            if not stage1_reward_check_trigger_topic or not stage2_reward_check_trigger_topic:
                raise RuntimeError(
                    "real_drb_dual_grasp_job: stage1/stage2 reward trigger topics "
                    "are required in policy mode"
                )

        # Resolve the base-start joint preset for the holding-robot trajectory seed.
        base_start = global_blackboard.pose_presets.get("base_start")
        if base_start is None:
            console.logerror("RealDrbDualGrasp: Missing pose preset [base_start]")
            return None
        base_start_left_joint_goal = base_start.get("left_joint_pos")
        base_start_right_joint_goal = base_start.get("right_joint_pos")
        if base_start_left_joint_goal is None or base_start_right_joint_goal is None:
            console.logerror(
                "RealDrbDualGrasp: Missing left/right joint preset in pose preset [base_start]"
            )
            return None
        base_start_holding_joint_goal = (
            base_start_left_joint_goal if holding_robot == left_robot else base_start_right_joint_goal
        )

        # Resolve the holding-robot regrasp-up preset used by the trajectory move.
        holding_robot_regrasp_up = global_blackboard.pose_presets.get("holding_robot_regrasp_up")
        if holding_robot_regrasp_up is None:
            console.logerror(
                "RealDrbDualGrasp: Missing pose preset [holding_robot_regrasp_up]"
            )
            return None
        holding_robot_regrasp_up_joint_goal = holding_robot_regrasp_up.get(
            "left_joint_pos" if holding_robot == left_robot else "right_joint_pos"
        )
        if holding_robot_regrasp_up_joint_goal is None:
            console.logerror(
                "RealDrbDualGrasp: Missing holding-robot joint preset in pose preset "
                "[holding_robot_regrasp_up]"
            )
            return None

        # Resolve above_mold_start joint targets.
        above_mold_start = global_blackboard.pose_presets.get("above_mold_start")
        if above_mold_start is None:
            console.logerror("RealDrbDualGrasp: Missing pose preset [above_mold_start]")
            return None
        above_mold_start_left_joint_goal = above_mold_start.get("left_joint_pos")
        above_mold_start_right_joint_goal = above_mold_start.get("right_joint_pos")
        if above_mold_start_left_joint_goal is None or above_mold_start_right_joint_goal is None:
            console.logerror("RealDrbDualGrasp: Missing left/right joint preset in pose preset [above_mold_start]")
            return None

        # Resolve both arm policy payloads before constructing policy subtrees.
        if does_policy_grasp:
            left_robot_policy_goal = self.make_policy_goal(
                step,
                left_robot,
                idx,
                default_timeout=MOVE_TIME,
            )
            right_robot_policy_goal = self.make_policy_goal(
                step,
                right_robot,
                idx,
                default_timeout=MOVE_TIME,
            )
            if left_robot_policy_goal is None or right_robot_policy_goal is None:
                raise RuntimeError(
                    "real_drb_dual_grasp_job: missing valid policy goals for policy mode"
                )

        # Estimate the regrasp target poses for both robots.
        pose_estimator = RingWorldModel.POSE_ESTIMATOR(
            name=plan_name,
            object_dict={"target": "dual_grasp_target"},
            robot_names=robot_names,
            holding_robot=holding_robot,
            approach_robot=approach_robot,
            tf_buffer=kwargs["tf_buffer"],
        )

        # Move the holding robot up and the approach robot down toward the regrasp target.
        move_approach_seq = py_trees.composites.Sequence(
            name="MoveApproachSeq",
            memory=True,
        )
        move_approach_right_parallel = MoveParallel.MoveParallel(
            name="ApproachRobotRegraspDownRightParallel"
        )
        move_holding_trajectory = MoveJoint.MOVEJT(
            name="HoldingRobotRegraspUpTrajectory",
            action_client=action_clients[holding_robot],
            action_goal=[
                base_start_holding_joint_goal,
                holding_robot_regrasp_up_joint_goal,
            ],
            robot_name=holding_robot,
            timeout=2*MOVE_TIME,
        )
        move_approach_right = self.make_move_pose_with_logger(
            name="ApproachRobotRegraspDownRight",
            action_client=action_clients[approach_robot],
            action_goal={"pose": plan_name + "/regrasp_target_down_right"},
            timeout=MOVE_TIME,
            robot_name=approach_robot,
            joint_logger_kwargs=joint_logger_kwargs,
        )
        move_approach_right_open = Gripper.GOTO(
            name="ApproachRobotGripperOpen",
            action_client=action_clients[approach_robot],
            action_goal=approach_robot_blackboard.gripper_open_pos,
            force=approach_robot_blackboard.gripper_open_force,
            timeout=GRIPPER_TIME,
            robot_name=approach_robot,
        )
        move_approach_right_parallel.add_children(
            [
                move_holding_trajectory,
                move_approach_right,
                move_approach_right_open,
            ]
        )
        move_approach = self.make_move_pose_with_logger(
            name="ApproachRobotRegraspDownHalfLeft",
            action_client=action_clients[approach_robot],
            action_goal={"pose": plan_name + "/real_regrasp_target_down_half_left"},
            timeout=MOVE_TIME*0.5,
            robot_name=approach_robot,
            joint_logger_kwargs=joint_logger_kwargs,
        )
        move_approach_wait = Wait.WAIT(
            name="WaitBeforeClose",
            duration=3.0,
            robot_name=approach_robot,
        )
        move_approach_wait_until_trigger = Wait.WAIT_UNTIL_TRIGGER(
            name="WaitUntilCloseTrigger",
            robot_name=approach_robot,
        )
        move_approach_close = Gripper.GOTO(
            name="ApproachRobotGripperClose",
            action_client=action_clients[approach_robot],
            action_goal=approach_robot_blackboard.gripper_close_pos,
            force=approach_robot_blackboard.gripper_close_force,
            timeout=GRIPPER_TIME,
            robot_name=approach_robot,
        )
        move_approach_seq.add_children(
            [
                move_approach_right_parallel,
                move_approach,
                move_approach_wait,
                # move_approach_wait_until_trigger,
                move_approach_close,
            ]
        )

        # Queue above_mold_start as one left/right parallel stage.
        above_mold_start_parallel = MoveParallel.MoveParallel(name="Preset_above_mold_start")
        above_mold_start_parallel.add_children(
            [
                MoveJoint.MOVEJ(
                    name=f"{left_robot}_above_mold_start",
                    action_client=action_clients[left_robot],
                    action_goal=above_mold_start_left_joint_goal,
                    robot_name=left_robot,
                    timeout=2*MOVE_TIME,
                ),
                MoveJoint.MOVEJ(
                    name=f"{right_robot}_above_mold_start",
                    action_client=action_clients[right_robot],
                    action_goal=above_mold_start_right_joint_goal,
                    robot_name=right_robot,
                    timeout=2*MOVE_TIME,
                ),
            ]
        )

        # Stop this real-hardware port after the initial approach sequence for now.
        root = py_trees.composites.Sequence(name="RealDrbDualGrasp", memory=True)
        root.add_children(
            [
                pose_estimator,
                move_approach_seq,
                above_mold_start_parallel
            ]
        )

        if does_policy_grasp:
            ### For test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            left_wait_until_trigger = Wait.WAIT_UNTIL_TRIGGER(
                name="LeftWaitUntilTrigger",
                robot_name=left_robot,
            )

            # Run stage1 placement on the left arm while only its reward helper is active.
            left_policy_seq = py_trees.composites.Sequence(
                name="LeftPandaPolicySeq",
                memory=True,
            )
            left_policy_switch_cartesian = RealControllerCommand.REAL_CONTROLLER_COMMAND(
                name="SwitchLeftCartesianBeforePlacementPolicy",
                command={
                    "action_type": "switchController",
                    "controller_profile": "cartesian_impedance_controller",
                    "target_arms": left_robot,
                },
                timeout=10.0,
            )
            left_run_policy = Policy.MOVEBYPOLICY(
                name=f"{left_robot}_PlacementPolicy",
                action_client=action_clients[left_robot],
                action_goal=left_robot_policy_goal,
                timeout=float(left_robot_policy_goal.get("timeout", MOVE_TIME)),
                robot_name=left_robot,
            )
            left_run_policy_with_reward_trigger = Trigger.RUN_WITH_BOOL_TRIGGER(
                name="Stage1PlacementRewardTrigger",
                child=left_run_policy,
                topic_name=stage1_reward_check_trigger_topic,
                start_value=True,
                stop_value=False,
            )
            left_policy_switch_jtc = RealControllerCommand.REAL_CONTROLLER_COMMAND(
                name="SwitchLeftJtcAfterPlacementPolicy",
                command={
                    "action_type": "switchController",
                    "controller_profile": "joint_trajectory_controller",
                    "target_arms": left_robot,
                },
                timeout=10.0,
            )
            left_policy_seq.add_children(
                [
                    left_policy_switch_cartesian,
                    left_run_policy_with_reward_trigger,
                    left_policy_switch_jtc,
                ]
            )

            # Run stage2 fit on the right arm while only its reward helper is active.
            right_policy_seq = py_trees.composites.Sequence(
                name="RightFr3PolicySeq",
                memory=True,
            )
            right_policy_switch_cartesian = RealControllerCommand.REAL_CONTROLLER_COMMAND(
                name="SwitchRightCartesianBeforeFitPolicy",
                command={
                    "action_type": "switchController",
                    "controller_profile": "cartesian_impedance_controller",
                    "target_arms": right_robot,
                },
                timeout=10.0,
            )
            right_run_policy = Policy.MOVEBYPOLICY(
                name=f"{right_robot}_FitPolicy",
                action_client=action_clients[right_robot],
                action_goal=right_robot_policy_goal,
                timeout=float(right_robot_policy_goal.get("timeout", MOVE_TIME)),
                robot_name=right_robot,
            )
            right_run_policy_with_reward_trigger = Trigger.RUN_WITH_BOOL_TRIGGER(
                name="Stage2FitRewardTrigger",
                child=right_run_policy,
                topic_name=stage2_reward_check_trigger_topic,
                start_value=True,
                stop_value=False,
            )
            right_policy_switch_jtc = RealControllerCommand.REAL_CONTROLLER_COMMAND(
                name="SwitchRightJtcAfterFitPolicy",
                command={
                    "action_type": "switchController",
                    "controller_profile": "joint_trajectory_controller",
                    "target_arms": right_robot,
                },
                timeout=10.0,
            )
            right_policy_seq.add_children(
                [
                    right_policy_switch_cartesian,
                    right_run_policy_with_reward_trigger,
                    right_policy_switch_jtc,
                ]
            )

            # Return both arms to base_start with JTC after both policy stages finish.
            policy_base_start_parallel = MoveParallel.MoveParallel(
                name="PolicyBaseStartParallel"
            )
            policy_base_start_parallel.add_children(
                [
                    MoveJoint.MOVEJ(
                        name=f"{left_robot}_PolicyBaseStart",
                        action_client=action_clients[left_robot],
                        action_goal=base_start_left_joint_goal,
                        robot_name=left_robot,
                        timeout=MOVE_TIME,
                    ),
                    MoveJoint.MOVEJ(
                        name=f"{right_robot}_PolicyBaseStart",
                        action_client=action_clients[right_robot],
                        action_goal=base_start_right_joint_goal,
                        robot_name=right_robot,
                        timeout=MOVE_TIME,
                    ),
                ]
            )
            root.add_children(
                [
                    left_wait_until_trigger,
                    left_policy_seq,
                    # right_policy_seq,
                    # policy_base_start_parallel,
                ]
            )
        return root
