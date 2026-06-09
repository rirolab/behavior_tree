import json

import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.validation_utils import StepValidationResult
from behavior_tree.subtrees import IsaacSceneCommand, Policy, RealControllerCommand


class Move(base_job.BaseJob):
    """
    Job handler for policy execution steps.
    """

    def __init__(self, node):
        super(Move, self).__init__(node)
        try:
            self.is_sim = bool(self._node.get_parameter("sim").value)
        except Exception:
            self.is_sim = True
        if self.is_sim:
            self.controller_command = IsaacSceneCommand.ISAAC_SCENE_COMMAND
            self._switch_controller_action_type = (
                "setRobotDriveGainProfileAndSwitchController"
            )
            self._switch_controller_profile_field = "robot_drive_gain_profile"
        else:
            self.controller_command = RealControllerCommand.REAL_CONTROLLER_COMMAND
            self._switch_controller_action_type = "switchController"
            self._switch_controller_profile_field = "controller_profile"

    def make_switch_controller_command(self, profile_name, target_arms):
        """
        Build one controller-switch command using the payload shape for the current runtime.

        Args:
            profile_name (:obj:`str`): target controller profile.
            target_arms: arm selector forwarded to the controller command behaviour.

        Returns:
            :obj:`dict`: controller-switch command payload.
        """
        return {
            "action_type": self._switch_controller_action_type,
            self._switch_controller_profile_field: profile_name,
            "target_arms": target_arms,
        }

    def acceptable_step(self, step):
        """
        Check whether this job should accept a grounding step for this primitive action.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        # Check if the primitive action is policy_execute
        if step.get("primitive_action") != "policy_execute":
            return False

        # Check if the step has the number of robots required for this job
        elif not self.check_robot_count(step, num_robot_required=1):
            return False

        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable policy step is well-formed enough to
        keep the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        # Check if the step has the required parameters for policy execution
        if self.acceptable_step(step):
            if bool(step.get("skill_id")):
                return StepValidationResult.ACCEPT_GOAL
            else:
                return StepValidationResult.REJECT_GOAL
        else:
            return StepValidationResult.NOT_APPLICABLE

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            self._node.get_logger().error("policy_job: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)["params"]
            for i in range(len(grounding.keys())):
                step = grounding.get(str(i + 1))
                if step is None:
                    continue
                if self.acceptable_step(step):
                    self.goal = grounding
                    break

    def create_root(self, action_client, idx="1", goal=std_msgs.Empty(), robot_name=None, **kwargs):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        # Check if the step is acceptable
        if not self.acceptable_step(goal[idx]):
            return None

        root = py_trees.composites.Sequence(name="Policy", memory=True)
        scene_cmd1 = self.controller_command(
            name="SwitchController1",
            command=self.make_switch_controller_command(
                "cartesian_impedance_controller",
                robot_name,
            ),
            timeout=10.0,
        )
        run_policy = Policy.MOVEBYPOLICY(
            name="MoveByPolicy",
            action_client=action_client,
            action_goal=goal[idx],
            timeout=float(goal[idx].get("timeout", 30.0)),
            robot_name=robot_name,
        )
        scene_cmd2 = self.controller_command(
            name="SwitchController2",
            command=self.make_switch_controller_command(
                "joint_trajectory_controller",
                robot_name,
            ),
            timeout=10.0,
        )
        root.add_children([scene_cmd1, run_policy, scene_cmd2])
        return root
