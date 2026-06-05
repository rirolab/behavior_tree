import json

import py_trees
import std_msgs.msg as std_msgs

from .. import base_job
from behavior_tree.subtrees import (
    IsaacSceneCommand,
    MoveJoint,
    MoveParallel,
    MovePose,
    Policy,
    RingWorldModel,
    Wait,
)
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Dual-arm test job that runs right-arm policy and left-arm goto in parallel.
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
        # Accept only dedicated two-arm test steps.
        if step.get("primitive_action") != "dual_policy_goto_test":
            return False
        elif not self.check_robot_count(step, num_robot_required=2):
            return False
        else:
            return True

    def resolve_left_right_robots(self, step):
        """
        Resolve the grounded left/right robot names from one step.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`tuple`: resolved ``(left_robot, right_robot)`` names.
        """
        # Resolve left/right robots from grounded robot list.
        grounded_robot_names = make_string_list(step.get("robot", []))
        left_robot = next((robot_name for robot_name in grounded_robot_names if "left" in robot_name), None)
        right_robot = next((robot_name for robot_name in grounded_robot_names if "right" in robot_name), None)
        return left_robot, right_robot

    def make_policy_goal(self, step, policy_robot, step_idx=None):
        """
        Build the right-arm policy goal from one shared multi-robot step.

        Args:
            step (:obj:`dict`): multi-robot grounding step.
            policy_robot (:obj:`str`): robot to build the policy goal for.
            step_idx (:obj:`str`): optional step index for policy execution.

        Returns:
            :obj:`dict`: robot-specific policy goal, or :obj:`None` if malformed.
        """
        # Build the robot-specific goal from shared and per-robot fields.
        policy_goal = self.make_robot_specific_goal(step, policy_robot, step_idx)
        if policy_goal is None:
            return None
        if policy_goal.get("implementation") != "policy":
            return None

        # Convert the robot-specific config into policy execution payload.
        policy_goal["primitive_action"] = "policy_execute"
        policy_goal["timeout"] = 30.0
        return policy_goal

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
        # Skip steps that belong to other jobs.
        if not self.acceptable_step(step):
            return StepValidationResult.NOT_APPLICABLE

        # Reject ambiguous shared implementation fields.
        if "implementation" in step:
            return StepValidationResult.REJECT_GOAL

        # Reject malformed or unknown left/right arm layouts.
        left_robot, right_robot = self.resolve_left_right_robots(step)
        bt_robot_names = getattr(self._node, "robot_names", [])
        if left_robot is None or right_robot is None or left_robot == right_robot:
            return StepValidationResult.REJECT_GOAL
        if bt_robot_names and (
            left_robot not in bt_robot_names or right_robot not in bt_robot_names
        ):
            return StepValidationResult.REJECT_GOAL

        # Reject malformed left-arm configuration.
        left_robot_goal = self.make_robot_specific_goal(step, left_robot, step.get("step_idx"))
        if left_robot_goal is None:
            return StepValidationResult.REJECT_GOAL

        # Reject missing right-arm policy configuration.
        policy_goal = self.make_policy_goal(step, right_robot, step.get("step_idx"))
        if policy_goal is None or not bool(policy_goal.get("skill_id")):
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
                "dual_policy_goto_test_job: rejecting new goal, previous still in the pipeline"
            )
        else:
            # Cache full grounding when one step belongs to this job.
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
        # Require robot list for dual-arm job creation.
        if robot_names is None:
            raise RuntimeError("dual_policy_goto_test_job: robot_names must be provided")

        # Ignore steps not owned by this job.
        if not self.acceptable_step(goal[idx]):
            return None

        # Resolve shared execution inputs from goal.
        step = goal[idx]
        left_robot, right_robot = self.resolve_left_right_robots(step)
        if left_robot is None or right_robot is None or left_robot == right_robot:
            raise RuntimeError("dual_policy_goto_test_job: expected one left robot and one right robot")
        action_clients = action_client
        plan_name = "Plan" + idx

        # Read left-arm init joints for primitive and return moves.
        left_robot_blackboard = py_trees.blackboard.Client(namespace=left_robot)
        left_robot_blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)
        # Read right-arm init joints for return move.
        right_robot_blackboard = py_trees.blackboard.Client(namespace=right_robot)
        right_robot_blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)

        # Build right-arm policy payload from shared step.
        policy_goal = self.make_policy_goal(step, right_robot, idx)
        if policy_goal is None or not bool(policy_goal.get("skill_id")):
            raise RuntimeError("dual_policy_goto_test_job: missing valid right-arm policy goal")

        # Publish regrasp target poses for primitive goto branch.
        pose_estimator = RingWorldModel.POSE_ESTIMATOR(
            name=plan_name,
            object_dict={"target": "dual_grasp_target"},
            robot_names=robot_names,
            holding_robot=left_robot,
            approach_robot=right_robot,
            tf_buffer=kwargs["tf_buffer"],
        )

        # Run left-arm primitive goto sequence with fixed timeouts.
        left_arm_goto_seq = py_trees.composites.Sequence(name="LeftArmGotoSeq", memory=True)
        left_arm_goto = MovePose.MOVEP(
            name=f"{left_robot}_RegraspTargetUp",
            action_client=action_clients[left_robot],
            action_goal={"pose": plan_name + "/regrasp_target_up"},
            timeout=3.0,
            robot_name=left_robot,
        )
        left_arm_goto_seq.add_children([left_arm_goto])

        # Run right-arm policy sequence with controller switching.
        right_arm_policy_seq = py_trees.composites.Sequence(name="RightArmPolicySeq", memory=True)
        switch_controller_in = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="RightArmSwitchController1",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "cartesian_impedance_controller",
                "target_arms": right_robot,
            },
            timeout=10.0,
        )
        run_policy = Policy.MOVEBYPOLICY(
            name=f"{right_robot}_MoveByPolicy",
            action_client=action_clients[right_robot],
            action_goal=policy_goal,
            timeout=30.0,
            robot_name=right_robot,
        )
        switch_controller_out = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="RightArmSwitchController2",
            command={
                "action_type": "setRobotDriveGainProfileAndSwitchController",
                "robot_drive_gain_profile": "joint_trajectory_controller",
                "target_arms": right_robot,
            },
            timeout=10.0,
        )
        right_arm_policy_seq.add_children([switch_controller_in, run_policy, switch_controller_out])

        # Run policy branch and primitive branch in parallel.
        run_parallel = MoveParallel.MoveParallel(name="TestDualPolicyGotoParallel")
        run_parallel.add_children([right_arm_policy_seq, left_arm_goto_seq])

        # Wait briefly after the parallel execution finishes.
        wait_after_parallel = Wait.WAIT(
            name="WaitAfterParallel",
            duration=2.0,
        )

        # Return both arms to init pose in parallel.
        return_init_parallel = MoveParallel.MoveParallel(name="ReturnInitParallel")
        return_left_init = MoveJoint.MOVEJ(
            name=f"{left_robot}_ReturnInitPose",
            action_client=action_clients[left_robot],
            action_goal=left_robot_blackboard.init_config,
            timeout=1.5,
            robot_name=left_robot,
        )
        return_right_init = MoveJoint.MOVEJ(
            name=f"{right_robot}_ReturnInitPose",
            action_client=action_clients[right_robot],
            action_goal=right_robot_blackboard.init_config,
            timeout=1.5,
            robot_name=right_robot,
        )
        return_init_parallel.add_children([return_left_init, return_right_init])

        # Estimate pose first, then run task, wait, and return together.
        root = py_trees.composites.Sequence(name="TestDualPolicyGoto", memory=True)
        root.add_children([pose_estimator, run_parallel, wait_after_parallel, return_init_parallel])
        return root
