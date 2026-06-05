import json

import py_trees
import std_msgs.msg as std_msgs

from .. import base_job
from behavior_tree.subtrees import RealControllerCommand, Wait
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Test job that switches both real arms to cartesian impedance without any motion,
    waits for observation, then switches both arms back to joint trajectory control.
    """

    def __init__(self, node):
        super(Move, self).__init__(node)
        try:
            self.is_sim = bool(self._node.get_parameter("sim").value)
        except Exception:
            self.is_sim = True

    def acceptable_step(self, step):
        """
        Check whether this job should accept the dual-arm switch-only experiment step.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        if self.is_sim:
            return False
        if step.get("primitive_action") != "real_dual_controller_switch_only_test":
            return False
        elif not self.check_robot_count(step, num_robot_required=2):
            return False
        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable dual-arm switch-only step is well-formed enough
        to keep the overall goal.

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
                "real_dual_controller_switch_only_test_job: rejecting new goal, previous still in the pipeline"
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
        OBSERVATION_WAIT_SEC = 10.0

        if not self.acceptable_step(goal[idx]):
            return None
        if robot_names is None:
            raise RuntimeError(
                "real_dual_controller_switch_only_test_job: robot_names must be provided"
            )

        grounded_robot_names = make_string_list(goal[idx].get("robot", []))
        if set(grounded_robot_names) != {"left_arm", "right_arm"}:
            raise RuntimeError(
                "real_dual_controller_switch_only_test_job: expected grounded robots [left_arm, right_arm]"
            )

        root = py_trees.composites.Sequence(
            name="RealDualControllerSwitchOnlyTest",
            memory=True,
        )

        # Ask the real bridge to perform one grouped switch:
        # deactivate both currently-active JTC owners and activate both CTC owners.
        switch_to_cartesian_impedance = RealControllerCommand.REAL_CONTROLLER_COMMAND(
            name="SwitchBothArmsToCartesianImpedanceOnly",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "cartesian_impedance_controller",
                "target_arms": grounded_robot_names,
            },
            timeout=10.0,
        )

        # Keep both CTCs active long enough to observe whether reflex appears.
        wait_after_switch = Wait.WAIT(
            name="WaitAfterDualCartesianImpedanceSwitchOnly",
            duration=OBSERVATION_WAIT_SEC,
        )

        # Return both arms to JTC through the same grouped controller-manager switch path.
        switch_to_joint_trajectory = RealControllerCommand.REAL_CONTROLLER_COMMAND(
            name="SwitchBothArmsBackToJointTrajectoryOnly",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "joint_trajectory_controller",
                "target_arms": grounded_robot_names,
            },
            timeout=10.0,
        )

        root.add_children(
            [
                switch_to_cartesian_impedance,
                wait_after_switch,
                switch_to_joint_trajectory,
            ]
        )
        return root
