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
    RingWorldModel,
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
        MOVE_TIME = 5.0
        GRIPPER_MOVE_TIME = 1.0

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
        does_manual_grasp = global_blackboard.drb_mode == "manual"

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

        # Resolve the second dual-arm preset used before the regrasp approach.
        stack_side_start = global_blackboard.pose_presets.get("stack_side_start2")
        if stack_side_start is None:
            console.logerror("RealDrbPick: Missing pose preset [stack_side_start]")
            return None
        stack_side_start_left_joint_goal = stack_side_start.get("left_joint_pos")
        stack_side_start_right_joint_goal = stack_side_start.get("right_joint_pos")
        if (
            stack_side_start_left_joint_goal is None
            or stack_side_start_right_joint_goal is None
        ):
            console.logerror(
                "RealDrbPick: Missing left/right joint preset in pose preset [stack_side_start]"
            )
            return None

        # Estimate the ring target frame that the right arm will approach.
        pose_estimator = RingWorldModel.POSE_ESTIMATOR(
            name=plan_name,
            object_dict={"target": "dual_grasp_target"},
            robot_names=robot_names,
            holding_robot=left_robot,
            approach_robot=right_robot,
            tf_buffer=kwargs["tf_buffer"],
        )

        # Replay the second stage with a left joint move and a right pose move plus logging.
        stack_side_start_parallel = MoveParallel.MoveParallel(name="StackSideStartParallel")
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
                ),
            ]
        )

        # Assemble the full pick sequence in execution order.
        root = py_trees.composites.Sequence(name="RealDrbPick", memory=True)
        root.add_children(
            [
                base_start_parallel,
                pose_estimator,
                stack_side_start_parallel,
            ]
        )

        if does_manual_grasp:
            # Read left-arm gripper parameters for the operator-assisted grasp step.
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
                    left_manual_grasp_move,
                    left_manual_grasp_open,
                    left_manual_grasp_wait,
                    left_manual_grasp_close,
                ]
            )

        return root
