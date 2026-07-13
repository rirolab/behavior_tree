import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import PolicyDual
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


# Robots a dual-arm policy step must name (order-insensitive).
DUAL_ARM_ROBOTS = {"left_arm", "right_arm"}


class Move(base_job.BaseJob):
    """
    Job handler for a single dual-arm policy execution step.

    Unlike dual_move_job (which runs an independent per-arm chain under a
    parallel composite), this job triggers ONE dual policy node as a single
    multi-robot step: it calls the shared ``dual_arm_client/command`` service
    and reads ``dual/goal_status`` back. No per-arm controller switching is
    performed (ffw_bg2 publishes to broadcasters directly).

    Grounding step schema::

        {
          "primitive_action": "dual_policy_execute",
          "robot": ["left_arm", "right_arm"],
          "policy_name": "<dual skill_id>",
          "timeout_sec": 30.0
        }
    """

    def __init__(self, node):
        super(Move, self).__init__(node)

    def acceptable_step(self, step):
        """
        Check whether this job should accept a grounding step for this primitive action.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        # Check if the primitive action is dual_policy_execute
        if step.get("primitive_action") != "dual_policy_execute":
            return False

        # Check if the step has the number of robots required for this job
        elif not self.check_robot_count(step, num_robot_required=2):
            return False

        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable dual policy step is well-formed enough to
        keep the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        # Ignore steps that are not dual policy executions.
        if step.get("primitive_action") != "dual_policy_execute":
            return StepValidationResult.NOT_APPLICABLE

        # The action matches: accept only if a policy is named and both arms are
        # requested, otherwise the step is malformed and rejects the goal.
        if bool(step.get("policy_name")) and (
            set(make_string_list(step.get("robot", []))) == DUAL_ARM_ROBOTS
        ):
            return StepValidationResult.ACCEPT_GOAL
        else:
            return StepValidationResult.REJECT_GOAL

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

        Called by multi_dynamic_behavior_tree's pre_tick_handler with the
        MULTI-robot signature (``action_client`` is a ``{robot_name: client}``
        mapping and ``robot_names`` is a list) because this step names two
        robots. The per-arm clients are intentionally ignored: a single dual
        policy node is triggered through the dual service client, which is
        plumbed in via the ``dual_action_client`` kwarg.

        Args:
            action_client (:obj:`dict`): per-arm command clients (ignored).
            idx (:obj:`str`): step index in the grounding plan.
            goal (:obj:`dict`): full grounding plan.
            robot_names ([:obj:`str`]): requested robot names (ignored).
            dual_action_client (:class:`~rclpy.client.Client`): dual service
                client on ``dual_arm_client/command`` (required, via kwargs).

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        # Check if the step is acceptable
        if not self.acceptable_step(goal[idx]):
            return None

        # Dual-arm policy goals are dispatched through the central policy
        # manager, not the dual_arm_client/command service.
        policy_action_client = kwargs.get("policy_action_client")
        if policy_action_client is None:
            self._node.get_logger().error(
                "dual_policy_job: no policy_action_client provided, cannot build subtree"
            )
            return None

        step = goal[idx]
        action_goal = {
            "skill_id": step["policy_name"],
            "timeout": step.get("timeout_sec", 30.0),
        }

        root = py_trees.composites.Sequence(name="PolicyDual", memory=True)
        run_policy = PolicyDual.MOVEBYPOLICYDUAL(
            name="MoveByPolicyDual",
            action_client=policy_action_client,
            action_goal=action_goal,
            timeout=float(step.get("timeout_sec", 30.0)),
            robot_name=None,
        )
        root.add_child(run_policy)
        return root
