import json

import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from .. import base_job
from behavior_tree.subtrees import MoveJoint, MoveParallel, RealControllerCommand, Wait
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Test job that exercises both-arm and per-arm real controller switching.
    """

    def __init__(self, node):
        super(Move, self).__init__(node)
        # Mirror node parameters onto the blackboard so pose presets and per-arm configs
        # can be reused in the same way as other real-hardware jobs.
        self.init_blackboard_parameters()
        try:
            self.is_sim = bool(self._node.get_parameter("sim").value)
        except Exception:
            self.is_sim = True

    def acceptable_step(self, step):
        """
        Check whether this job should accept a dedicated real controller switch step.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        if self.is_sim:
            return False
        if step.get("primitive_action") != "real_controller_switch_test":
            return False
        elif not self.check_robot_count(step, num_robot_required=2):
            return False
        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable controller-switch step is well-formed enough to
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
        if len(robot_names) != 2:
            return StepValidationResult.REJECT_GOAL
        if set(robot_names) != {"left_arm", "right_arm"}:
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
                "real_controller_switch_test_job: rejecting new goal, previous still in the pipeline"
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

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        MOVE_TIME = 5.0

        if not self.acceptable_step(goal[idx]):
            return None
        if robot_names is None:
            raise RuntimeError(
                "real_controller_switch_test_job: robot_names must be provided"
            )
        # Multi-robot jobs receive one action-client mapping keyed by robot name.
        if not isinstance(action_client, dict):
            raise RuntimeError(
                "real_controller_switch_test_job: action_clients mapping must be provided"
            )

        # Resolve the grounded left/right arm names once for all later subtree blocks.
        grounded_robot_names = make_string_list(goal[idx].get("robot", []))
        if set(grounded_robot_names) != {"left_arm", "right_arm"}:
            raise RuntimeError(
                "real_controller_switch_test_job: expected grounded robots [left_arm, right_arm]"
            )
        left_robot = next(
            (robot_name for robot_name in grounded_robot_names if "left" in robot_name),
            None,
        )
        right_robot = next(
            (robot_name for robot_name in grounded_robot_names if "right" in robot_name),
            None,
        )
        if left_robot is None or right_robot is None:
            raise RuntimeError(
                "real_controller_switch_test_job: expected grounded left/right robots"
            )

        # Read the shared base_start preset from the global blackboard.
        global_blackboard = py_trees.blackboard.Client()
        global_blackboard.register_key(
            key="pose_presets",
            access=py_trees.common.Access.READ,
        )
        base_start = global_blackboard.pose_presets.get("base_start")
        if base_start is None:
            console.logerror(
                "real_controller_switch_test_job: Missing pose preset [base_start]"
            )
            return None
        base_start_left_joint_goal = base_start.get("left_joint_pos")
        base_start_right_joint_goal = base_start.get("right_joint_pos")
        if base_start_left_joint_goal is None or base_start_right_joint_goal is None:
            console.logerror(
                "real_controller_switch_test_job: Missing left/right joint preset in pose preset [base_start]"
            )
            return None

        # Read each arm's init_config so the test can return to the configured home pose.
        left_blackboard = py_trees.blackboard.Client(namespace=left_robot)
        left_blackboard.register_key(
            key="init_config",
            access=py_trees.common.Access.READ,
        )
        right_blackboard = py_trees.blackboard.Client(namespace=right_robot)
        right_blackboard.register_key(
            key="init_config",
            access=py_trees.common.Access.READ,
        )
        if left_blackboard.init_config is None or right_blackboard.init_config is None:
            console.logerror(
                "real_controller_switch_test_job: Missing left/right init_config"
            )
            return None

        root = py_trees.composites.Sequence(
            name="RealControllerSwitchTest",
            memory=True,
        )

        def make_switch_command(name, controller_profile, target_arms):
            """
            Build one real-controller bridge command for one arm or both arms.
            """
            return RealControllerCommand.REAL_CONTROLLER_COMMAND(
                name=name,
                command={
                    "action_type": "switchController",
                    "controller_profile": controller_profile,
                    "target_arms": target_arms,
                },
                timeout=10.0,
            )

        def make_move_joint(name, robot_name, action_goal):
            """
            Build one joint move for a single arm.
            """
            return MoveJoint.MOVEJ(
                name=name,
                action_client=action_client[robot_name],
                action_goal=action_goal,
                robot_name=robot_name,
                timeout=MOVE_TIME,
            )

        def make_wait(name):
            """
            Build one fixed wait used after controller switches.
            """
            return Wait.WAIT(
                name=name,
                duration=5.0,
            )

        # Move both arms to the shared base_start preset before any controller switching.
        base_start_parallel = MoveParallel.MoveParallel(name="BaseStartParallel")
        base_start_parallel.add_children(
            [
                make_move_joint(
                    name=f"{left_robot}_BaseStart",
                    robot_name=left_robot,
                    action_goal=base_start_left_joint_goal,
                ),
                make_move_joint(
                    name=f"{right_robot}_BaseStart",
                    robot_name=right_robot,
                    action_goal=base_start_right_joint_goal,
                ),
            ]
        )

        # Switch both arms from JTC to cartesian impedance through the real bridge.
        switch_to_cartesian_impedance = make_switch_command(
            name="SwitchBothArmsToCartesianImpedance",
            controller_profile="cartesian_impedance_controller",
            target_arms=grounded_robot_names,
        )

        # Hold the cartesian impedance mode briefly so the switch can be observed.
        wait_after_switch = make_wait("WaitAfterCartesianImpedanceSwitch")

        # Switch both arms back onto their joint trajectory controllers.
        switch_to_joint_trajectory = make_switch_command(
            name="SwitchBothArmsToJointTrajectory",
            controller_profile="joint_trajectory_controller",
            target_arms=grounded_robot_names,
        )
        wait_after_joint_trajectory_switch = make_wait(
            "WaitAfterJointTrajectorySwitch"
        )

        # Return both arms to their configured init poses after the controller test.
        init_parallel = MoveParallel.MoveParallel(name="InitParallel")
        init_parallel.add_children(
            [
                make_move_joint(
                    name=f"{left_robot}_Init",
                    robot_name=left_robot,
                    action_goal=left_blackboard.init_config,
                ),
                make_move_joint(
                    name=f"{right_robot}_Init",
                    robot_name=right_robot,
                    action_goal=right_blackboard.init_config,
                ),
            ]
        )

        # After both arms return to init, cycle each arm individually before parking it.
        left_arm_switch_cycle = py_trees.composites.Sequence(
            name="LeftArmJtcToCtcToJtc",
            memory=True,
        )
        left_arm_switch_cycle.add_children(
            [
                make_switch_command(
                    name="SwitchLeftArmToCartesianImpedance",
                    controller_profile="cartesian_impedance_controller",
                    target_arms=left_robot,
                ),
                make_wait("WaitAfterLeftArmCartesianImpedanceSwitch"),
                make_switch_command(
                    name="SwitchLeftArmToJointTrajectory",
                    controller_profile="joint_trajectory_controller",
                    target_arms=left_robot,
                )
            ]
        )

        left_arm_base_start = make_move_joint(
            name=f"{left_robot}_BaseStartAgain",
            robot_name=left_robot,
            action_goal=base_start_left_joint_goal,
        )

        right_arm_switch_cycle = py_trees.composites.Sequence(
            name="RightArmJtcToCtcToJtc",
            memory=True,
        )
        right_arm_switch_cycle.add_children(
            [
                make_switch_command(
                    name="SwitchRightArmToCartesianImpedance",
                    controller_profile="cartesian_impedance_controller",
                    target_arms=right_robot,
                ),
                make_wait("WaitAfterRightArmCartesianImpedanceSwitch"),
                make_switch_command(
                    name="SwitchRightArmToJointTrajectory",
                    controller_profile="joint_trajectory_controller",
                    target_arms=right_robot,
                )
            ]
        )

        right_arm_base_start = make_move_joint(
            name=f"{right_robot}_BaseStartAgain",
            robot_name=right_robot,
            action_goal=base_start_right_joint_goal,
        )

        # Execute the full controller-switch smoke test as one ordered sequence.
        root.add_children(
            [
                base_start_parallel,
                switch_to_cartesian_impedance,
                wait_after_switch,
                switch_to_joint_trajectory,
                wait_after_joint_trajectory_switch,
                init_parallel,
                left_arm_switch_cycle,
                left_arm_base_start,
                right_arm_switch_cycle,
                right_arm_base_start,
            ]
        )
        return root
