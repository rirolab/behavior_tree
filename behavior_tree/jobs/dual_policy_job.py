import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import IsaacSceneCommand, MoveParallel, Policy, RealControllerCommand
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Dual-arm policy execution job.
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
        if step.get("primitive_action") != "policy_execute":
            return False
        elif not self.check_robot_count(step, num_robot_required=2):
            return False
        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable dual-arm policy step is well-formed enough to
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
        if len(robot_names) != 2 or len(set(robot_names)) != 2:
            return StepValidationResult.REJECT_GOAL

        # Require per-robot policy configuration on both arms.
        for robot_name in robot_names:
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
                "dual_policy_job: rejecting new goal, previous still in the pipeline"
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
        if robot_names is None:
            raise RuntimeError("dual_policy_job: robot_names must be provided")

        if not self.acceptable_step(goal[idx]):
            return None

        step = goal[idx]
        grounded_robot_names = make_string_list(step.get("robot", []))

        if len(grounded_robot_names) != 2:
            raise RuntimeError("dual_policy_job: expected exactly two robots in the grounding")

        scene_cmd1 = self.controller_command(
            name="DualSwitchController1",
            command=self.make_switch_controller_command(
                "cartesian_impedance_controller",
                grounded_robot_names,
            ),
            timeout=10.0,
        )

        run_policy_parallel = MoveParallel.MoveParallel(name="RunPolicyParallel")
        # Build one policy branch per robot from robot-specific payloads.
        for robot_name in grounded_robot_names:
            robot_goal = self.make_robot_specific_goal(step, robot_name, idx)
            if not bool(robot_goal.get("skill_id")):
                raise RuntimeError(
                    f"dual_policy_job: missing valid policy goal for robot [{robot_name}]"
                )

            run_policy = Policy.MOVEBYPOLICY(
                name=f"{robot_name}_MoveByPolicy",
                action_client=action_client[robot_name],
                action_goal=robot_goal,
                timeout=float(robot_goal.get("timeout", 5.0)),
                robot_name=robot_name,
            )
            run_policy_parallel.add_child(run_policy)

        scene_cmd2 = self.controller_command(
            name="DualSwitchController2",
            command=self.make_switch_controller_command(
                "joint_trajectory_controller",
                grounded_robot_names,
            ),
            timeout=10.0,
        )

        root = py_trees.composites.Sequence(name="DualPolicy", memory=True)
        root.add_children([scene_cmd1, run_policy_parallel, scene_cmd2])
        return root
