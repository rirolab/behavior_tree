import json

import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from .. import base_job
from behavior_tree.subtrees import MoveJoint, MoveParallel, Policy, RealControllerCommand
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


PRIMITIVE_ACTION = "real_hil_serl_jtc_sequence_test"
MOVE_TIME = 5.0


class Move(base_job.BaseJob):
    """
    Real dual-arm test job that alternates JTC moves and HIL-SERL policy moves.
    """

    def __init__(self, node):
        super(Move, self).__init__(node)
        # Mirror node parameters onto the blackboard for pose presets and arm configs.
        self.init_blackboard_parameters()
        try:
            self.is_sim = bool(self._node.get_parameter("sim").value)
        except Exception:
            self.is_sim = True

    def acceptable_step(self, step):
        """
        Check whether this job should accept the real HIL-SERL/JTC sequence step.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        # Accept only the dedicated real two-arm test primitive.
        if self.is_sim:
            return False
        if step.get("primitive_action") != PRIMITIVE_ACTION:
            return False
        elif not self.check_robot_count(step, num_robot_required=2):
            return False
        else:
            return True

    def resolve_left_right_robots(self, step):
        """
        Resolve the grounded left/right robot names from one test step.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`tuple`: resolved ``(left_robot, right_robot)`` names.
        """
        # Split the grounded robot list into the expected left/right arms.
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

    def make_policy_goal(self, step, robot_name, step_idx=None):
        """
        Build one HIL-SERL policy goal for the requested arm.

        Args:
            step (:obj:`dict`): shared multi-robot grounding step.
            robot_name (:obj:`str`): robot whose policy block should be used.
            step_idx (:obj:`str`): optional step index for policy caching.

        Returns:
            :obj:`dict`: robot-specific policy goal, or :obj:`None` if malformed.
        """
        # Merge shared fields with the robot-specific HIL-SERL skill config.
        policy_goal = self.make_robot_specific_goal(step, robot_name, step_idx)
        if policy_goal is None:
            return None
        if not bool(policy_goal.get("skill_id")):
            return None

        # Force the generated payload into the arm-client policy execution path.
        policy_goal["primitive_action"] = "policy_execute"
        try:
            policy_goal["timeout"] = float(policy_goal.get("timeout", MOVE_TIME))
        except (TypeError, ValueError):
            return None
        return policy_goal

    def make_policy_preload_requests(self, step, step_idx):
        """
        Build preload requests for both HIL-SERL policies used by this sequence.

        Args:
            step (:obj:`dict`): shared multi-robot grounding step.
            step_idx (:obj:`str`): step index in the grounding plan.

        Returns:
            [(:obj:`str`, :obj:`dict`)]: robot-specific preload requests.
        """
        # Preload both arm policies before any controller switches or movements.
        if not self.acceptable_step(step):
            return []
        requests = []
        for robot_name in make_string_list(step.get("robot", [])):
            policy_goal = self.make_policy_goal(step, robot_name, step_idx)
            if policy_goal is None:
                return []
            requests.append((robot_name, policy_goal))
        return requests

    def validate_step(self, step):
        """
        Validate whether this test step is well-formed enough to keep the goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: validation outcome for this job.
        """
        # Ignore steps that belong to other job handlers.
        if not self.acceptable_step(step):
            return StepValidationResult.NOT_APPLICABLE

        # Require exactly one configured left arm and one configured right arm.
        left_robot, right_robot = self.resolve_left_right_robots(step)
        bt_robot_names = getattr(self._node, "robot_names", [])
        if left_robot is None or right_robot is None or left_robot == right_robot:
            return StepValidationResult.REJECT_GOAL
        if bt_robot_names and (
            left_robot not in bt_robot_names or right_robot not in bt_robot_names
        ):
            return StepValidationResult.REJECT_GOAL

        # Require one valid HIL-SERL skill config per arm.
        if self.make_policy_goal(step, left_robot, step.get("step_idx")) is None:
            return StepValidationResult.REJECT_GOAL
        if self.make_policy_goal(step, right_robot, step.get("step_idx")) is None:
            return StepValidationResult.REJECT_GOAL
        return StepValidationResult.ACCEPT_GOAL

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message.
        """
        # Cache the full grounding when one step belongs to this job.
        if self.goal:
            self._node.get_logger().error(
                "real_hil_serl_jtc_sequence_test_job: rejecting new goal, previous still in the pipeline"
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

    def make_switch_command(self, name, controller_profile, target_arms):
        """
        Build one real-controller bridge command.

        Args:
            name (:obj:`str`): behaviour name.
            controller_profile (:obj:`str`): target controller profile.
            target_arms: arm selector forwarded to the controller bridge.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: controller switch behaviour.
        """
        # Send controller switches only as standalone sequence steps.
        return RealControllerCommand.REAL_CONTROLLER_COMMAND(
            name=name,
            command={
                "action_type": "switchController",
                "controller_profile": controller_profile,
                "target_arms": target_arms,
            },
            timeout=10.0,
        )

    def make_move_joint(self, name, action_clients, robot_name, action_goal):
        """
        Build one 5-second joint-trajectory movement.

        Args:
            name (:obj:`str`): behaviour name.
            action_clients (:obj:`dict`): action client mapping by robot.
            robot_name (:obj:`str`): arm to command.
            action_goal: joint target list.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: joint movement behaviour.
        """
        # Run JTC moves through the arm client's moveJoint path.
        return MoveJoint.MOVEJ(
            name=name,
            action_client=action_clients[robot_name],
            action_goal=action_goal,
            timeout=MOVE_TIME,
            robot_name=robot_name,
        )

    def make_policy_move(self, name, action_clients, robot_name, action_goal):
        """
        Build one HIL-SERL policy movement using the policy payload timeout.

        Args:
            name (:obj:`str`): behaviour name.
            action_clients (:obj:`dict`): action client mapping by robot.
            robot_name (:obj:`str`): arm to command.
            action_goal (:obj:`dict`): policy execution payload.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: policy movement behaviour.
        """
        # Run HIL-SERL moves through the arm client's moveByPolicy path.
        return Policy.MOVEBYPOLICY(
            name=name,
            action_client=action_clients[robot_name],
            action_goal=action_goal,
            timeout=float(action_goal.get("timeout", MOVE_TIME)),
            robot_name=robot_name,
        )

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
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification.

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root.
        """
        # Ignore steps not owned by this job.
        if not self.acceptable_step(goal[idx]):
            return None
        if robot_names is None:
            raise RuntimeError(
                "real_hil_serl_jtc_sequence_test_job: robot_names must be provided"
            )
        if not isinstance(action_client, dict):
            raise RuntimeError(
                "real_hil_serl_jtc_sequence_test_job: action_clients mapping must be provided"
            )

        # Resolve the two arm names used by the ordered test sequence.
        step = goal[idx]
        action_clients = action_client
        grounded_robot_names = make_string_list(step.get("robot", []))
        left_robot, right_robot = self.resolve_left_right_robots(step)
        if left_robot is None or right_robot is None or left_robot == right_robot:
            raise RuntimeError(
                "real_hil_serl_jtc_sequence_test_job: expected one left robot and one right robot"
            )

        # Read the shared base_start joint targets from pose presets.
        global_blackboard = py_trees.blackboard.Client()
        global_blackboard.register_key(
            key="pose_presets",
            access=py_trees.common.Access.READ,
        )
        try:
            pose_presets = global_blackboard.pose_presets
        except KeyError:
            console.logerror(
                "real_hil_serl_jtc_sequence_test_job: Missing pose_presets"
            )
            return None
        base_start = pose_presets.get("base_start")
        if base_start is None:
            console.logerror(
                "real_hil_serl_jtc_sequence_test_job: Missing pose preset [base_start]"
            )
            return None
        base_start_left_joint_goal = base_start.get("left_joint_pos")
        base_start_right_joint_goal = base_start.get("right_joint_pos")
        if base_start_left_joint_goal is None or base_start_right_joint_goal is None:
            console.logerror(
                "real_hil_serl_jtc_sequence_test_job: Missing left/right joint preset in [base_start]"
            )
            return None

        # Read each arm's init_config for the repeated JTC return phases.
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
                "real_hil_serl_jtc_sequence_test_job: Missing left/right init_config"
            )
            return None

        # Build policy goals once and reuse them for the repeated policy phases.
        left_policy_goal = self.make_policy_goal(step, left_robot, idx)
        right_policy_goal = self.make_policy_goal(step, right_robot, idx)
        if left_policy_goal is None or right_policy_goal is None:
            raise RuntimeError(
                "real_hil_serl_jtc_sequence_test_job: missing valid HIL-SERL policy goals"
            )

        # Phase 1 moves left arm with JTC and right arm with HIL-SERL policy.
        left_base_right_policy = MoveParallel.MoveParallel(
            name="LeftBaseStartRightPolicy"
        )
        left_base_right_policy.add_children(
            [
                self.make_move_joint(
                    name=f"{left_robot}_BaseStart",
                    action_clients=action_clients,
                    robot_name=left_robot,
                    action_goal=base_start_left_joint_goal,
                ),
                self.make_policy_move(
                    name=f"{right_robot}_PolicyOffset1",
                    action_clients=action_clients,
                    robot_name=right_robot,
                    action_goal=right_policy_goal,
                ),
            ]
        )

        # Phase 2 returns both arms to init with JTC.
        both_init_after_right_policy = MoveParallel.MoveParallel(
            name="BothInitAfterRightPolicy"
        )
        both_init_after_right_policy.add_children(
            [
                self.make_move_joint(
                    name=f"{left_robot}_InitAfterRightPolicy",
                    action_clients=action_clients,
                    robot_name=left_robot,
                    action_goal=left_blackboard.init_config,
                ),
                self.make_move_joint(
                    name=f"{right_robot}_InitAfterRightPolicy",
                    action_clients=action_clients,
                    robot_name=right_robot,
                    action_goal=right_blackboard.init_config,
                ),
            ]
        )

        # Phase 3 moves left arm with HIL-SERL policy and right arm with JTC.
        left_policy_right_base = MoveParallel.MoveParallel(
            name="LeftPolicyRightBaseStart"
        )
        left_policy_right_base.add_children(
            [
                self.make_policy_move(
                    name=f"{left_robot}_PolicyOffset1",
                    action_clients=action_clients,
                    robot_name=left_robot,
                    action_goal=left_policy_goal,
                ),
                self.make_move_joint(
                    name=f"{right_robot}_BaseStart",
                    action_clients=action_clients,
                    robot_name=right_robot,
                    action_goal=base_start_right_joint_goal,
                ),
            ]
        )

        # Phase 4 returns both arms to init with JTC.
        both_init_after_left_policy = MoveParallel.MoveParallel(
            name="BothInitAfterLeftPolicy"
        )
        both_init_after_left_policy.add_children(
            [
                self.make_move_joint(
                    name=f"{left_robot}_InitAfterLeftPolicy",
                    action_clients=action_clients,
                    robot_name=left_robot,
                    action_goal=left_blackboard.init_config,
                ),
                self.make_move_joint(
                    name=f"{right_robot}_InitAfterLeftPolicy",
                    action_clients=action_clients,
                    robot_name=right_robot,
                    action_goal=right_blackboard.init_config,
                ),
            ]
        )

        # Phase 5 moves both arms with HIL-SERL policies.
        both_policy = MoveParallel.MoveParallel(name="BothPolicyOffset")
        both_policy.add_children(
            [
                self.make_policy_move(
                    name=f"{left_robot}_PolicyOffset2",
                    action_clients=action_clients,
                    robot_name=left_robot,
                    action_goal=left_policy_goal,
                ),
                self.make_policy_move(
                    name=f"{right_robot}_PolicyOffset2",
                    action_clients=action_clients,
                    robot_name=right_robot,
                    action_goal=right_policy_goal,
                ),
            ]
        )

        # Sequence controller switches separately from all movement phases.
        root = py_trees.composites.Sequence(
            name="RealHilSerlJtcSequenceTest",
            memory=True,
        )
        root.add_children(
            [
                # Start from both arms already on JTC, then switch only the policy arm.
                self.make_switch_command(
                    name="SwitchRightCartesianBeforeRightPolicy",
                    controller_profile="cartesian_impedance_controller",
                    target_arms=right_robot,
                ),
                left_base_right_policy,
                # Return only the right arm to JTC before the dual init JTC move.
                self.make_switch_command(
                    name="SwitchRightJtcBeforeInit1",
                    controller_profile="joint_trajectory_controller",
                    target_arms=right_robot,
                ),
                both_init_after_right_policy,
                # Switch only the left arm to CIC for the next policy/JTC parallel phase.
                self.make_switch_command(
                    name="SwitchLeftCartesianBeforeLeftPolicy",
                    controller_profile="cartesian_impedance_controller",
                    target_arms=left_robot,
                ),
                left_policy_right_base,
                # Return only the left arm to JTC before the second dual init JTC move.
                self.make_switch_command(
                    name="SwitchLeftJtcBeforeInit2",
                    controller_profile="joint_trajectory_controller",
                    target_arms=left_robot,
                ),
                both_init_after_left_policy,
                # Switch both arms to CIC together before the final dual-policy phase.
                self.make_switch_command(
                    name="SwitchBothCartesianBeforeBothPolicy",
                    controller_profile="cartesian_impedance_controller",
                    target_arms=grounded_robot_names,
                ),
                both_policy,
                # Restore both arms to JTC after the test finishes.
                self.make_switch_command(
                    name="SwitchBothJtcAfterBothPolicy",
                    controller_profile="joint_trajectory_controller",
                    target_arms=grounded_robot_names,
                ),
            ]
        )
        return root
