import json

import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MoveJoint, RealControllerCommand, Wait
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Test job that switches only the left arm to cartesian impedance, waits, then switches back.
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
        Check whether this job should accept a dedicated left-only real controller switch step.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        if self.is_sim:
            return False
        if step.get("primitive_action") != "real_left_controller_switch_test":
            return False
        elif not self.check_robot_count(step, num_robot_required=1):
            return False
        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable left-only controller-switch step is well-formed enough to
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
        if len(robot_names) != 1:
            return StepValidationResult.REJECT_GOAL
        if robot_names[0] != "left_arm":
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
                "real_left_controller_switch_test_job: rejecting new goal, previous still in the pipeline"
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
        robot_name=None,
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

        # Resolve the grounded left arm name once for all later subtree blocks.
        grounded_robot_names = make_string_list(goal[idx].get("robot", []))
        if len(grounded_robot_names) != 1 or grounded_robot_names[0] != "left_arm":
            raise RuntimeError(
                "real_left_controller_switch_test_job: expected grounded robot [left_arm]"
            )
        left_robot = robot_name or grounded_robot_names[0]
        if left_robot != "left_arm":
            raise RuntimeError(
                "real_left_controller_switch_test_job: expected grounded left robot"
            )
        # Accept the standard single-robot action client path, while also tolerating
        # a robot_name -> client mapping if the caller provides one.
        left_action_client = (
            action_client[left_robot] if isinstance(action_client, dict) else action_client
        )

        # Read the shared base_start preset and keep only the left-arm joint goal.
        global_blackboard = py_trees.blackboard.Client()
        global_blackboard.register_key(
            key="pose_presets",
            access=py_trees.common.Access.READ,
        )
        base_start = global_blackboard.pose_presets.get("base_start")
        if base_start is None:
            console.logerror(
                "real_left_controller_switch_test_job: Missing pose preset [base_start]"
            )
            return None
        base_start_left_joint_goal = base_start.get("left_joint_pos")
        if base_start_left_joint_goal is None:
            console.logerror(
                "real_left_controller_switch_test_job: Missing left joint preset in pose preset [base_start]"
            )
            return None

        # Read the left arm init_config so the test can return the same arm home afterward.
        left_blackboard = py_trees.blackboard.Client(namespace=left_robot)
        left_blackboard.register_key(
            key="init_config",
            access=py_trees.common.Access.READ,
        )
        if left_blackboard.init_config is None:
            console.logerror(
                "real_left_controller_switch_test_job: Missing left init_config"
            )
            return None

        root = py_trees.composites.Sequence(
            name="RealLeftControllerSwitchTest",
            memory=True,
        )

        # Move only the left arm to the shared base_start preset before controller switching.
        move_left_to_base_start = MoveJoint.MOVEJ(
            name=f"{left_robot}_BaseStart",
            action_client=left_action_client,
            action_goal=base_start_left_joint_goal,
            robot_name=left_robot,
            timeout=MOVE_TIME,
        )

        # Switch only the left arm from JTC to cartesian impedance through the real bridge.
        switch_left_to_cartesian_impedance = RealControllerCommand.REAL_CONTROLLER_COMMAND(
            name="SwitchLeftArmToCartesianImpedance",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "cartesian_impedance_controller",
                "target_arms": "left_arm",
            },
            timeout=10.0,
        )

        # Hold the left cartesian impedance mode briefly so the switch can be observed.
        wait_after_switch = Wait.WAIT(
            name="WaitAfterLeftCartesianImpedanceSwitch",
            duration=10.0,
        )

        # Switch only the left arm back onto its joint trajectory controller.
        switch_left_to_joint_trajectory = RealControllerCommand.REAL_CONTROLLER_COMMAND(
            name="SwitchLeftArmToJointTrajectory",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "joint_trajectory_controller",
                "target_arms": "left_arm",
            },
            timeout=10.0,
        )

        # Return only the left arm to its configured init pose after the controller test.
        move_left_to_init = MoveJoint.MOVEJ(
            name=f"{left_robot}_Init",
            action_client=left_action_client,
            action_goal=left_blackboard.init_config,
            robot_name=left_robot,
            timeout=MOVE_TIME,
        )

        # Execute the full left-only controller-switch smoke test as one ordered sequence.
        root.add_children(
            [
                move_left_to_base_start,
                switch_left_to_cartesian_impedance,
                wait_after_switch,
                switch_left_to_joint_trajectory,
                move_left_to_init,
            ]
        )
        return root
