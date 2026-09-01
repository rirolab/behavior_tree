import json

import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.validation_utils import StepValidationResult
from behavior_tree.subtrees import Policy


class Move(base_job.BaseJob):
    """
    Job handler for policy execution steps.
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

        # Policy goals are dispatched through the central policy manager, not
        # the per-arm arm_client/command service positionally passed in.
        policy_action_client = kwargs.get("policy_action_client")
        if policy_action_client is None:
            self._node.get_logger().error(
                "policy_job: no policy_action_client provided, cannot build subtree"
            )
            return None

        # Built by the subtree module rather than inline, so the policy step
        # has one shape: an OverlapSequence root that reports the policy's
        # progress upward, which is what lets the sequence chaining the steps
        # blend a primitive into this step and this step into the next.
        return Policy.create_subtree(
            policy_action_client, goal[idx], robot_name=robot_name
        )
