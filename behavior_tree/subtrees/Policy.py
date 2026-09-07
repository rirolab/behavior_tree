import json

import numpy as np
import py_trees
from action_msgs.msg import GoalStatus

from riro_srvs.srv import StringGoalStatus

from . import Move
from .MoveBlend import (
    OVERLAP_DISPATCH_ON_START_PARAM,
    OVERLAP_THRESHOLD_PARAM,
    MoveBlend,
)


class MOVEBYPOLICY(Move.MOVE):
    """
    Execute policy through complex_action_client.
    """

    def __init__(self, name, action_client, action_goal=None, timeout=5.0, robot_name=None):
        """
        Initialise a policy execution behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            action_client (:class:`~rclpy.client.Client`): robot command client.
            action_goal (:obj:`dict`): policy command payload.
            timeout (:obj:`float`): command timeout in seconds.
            robot_name (:obj:`str`): optional robot namespace.
        """
        super(MOVEBYPOLICY, self).__init__(
            name=name,
            action_client=action_client,
            action_goal=action_goal,
            timeout=timeout,
            robot_name=robot_name,
        )

        # A policy step CAN be overlapped, but not the way a primitive is.
        #
        # Its status comes on the plain arm channel: only one policy goal is
        # ever in flight, so there is nothing to disambiguate between slots and
        # Move.MOVE's slot matching would find nothing. Its *command*, though,
        # goes through a mixer slot exactly like a streamed primitive, so the
        # motion is blendable and the sequence above may start the next step
        # before this one finishes.
        self.stream_slots = ()
        self.overlappable = True

        # And progress comes from the policy executor rather than arm_client:
        # `step / max_steps`, which is bounded and monotonic. Its own key, not
        # one of the arm slots' -- arm_client owns those, and two writers on one
        # key would race whenever a primitive and a policy are live together,
        # which under overlap is the whole point.
        self.progress_key = "policy/progress"
        self.blackboard.register_key(
            key=self.progress_key,
            access=py_trees.common.Access.READ,
        )

        self.logger.debug("%s.__init__()" % self.__class__.__name__)

    def current_progress(self):
        """Fraction of the policy's motion completed, in [0, 1], or None.

        None until the executor publishes -- before that the sequence above
        cannot overlap this step and falls back to sequential, which is the
        safe reading of "we do not know how far along it is".
        """
        return self._read(self.progress_key)

    def update(self):
        """
        Send the policy command and wait for the matching goal status.

        Returns:
            :class:`~py_trees.common.Status`: behaviour status.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16, dtype=np.uint8)
            cmd_str = json.dumps(
                {
                    "action_type": "moveByPolicy",
                    "goal": self.action_goal,
                    "uuid": self.goal_uuid_des.tolist(),
                    "timeout": self.timeout,
                    "enable_wait": False,
                }
            )
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)

            self.sent_goal = True
            self.feedback_message = "Sending a policy goal"
            return py_trees.common.Status.RUNNING

        if self.current_goal_id() is None:
            return py_trees.common.Status.RUNNING

        if not self.goal_matches_blackboard():
            return py_trees.common.Status.RUNNING

        status = self.current_goal_status()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.SUCCESS, \
                                   self.feedback_message))
            return py_trees.common.Status.SUCCESS

        if status in [
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_CANCELED,
        ]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                   self.feedback_message))
            return py_trees.common.Status.FAILURE

        self.feedback_message = "running"
        return py_trees.common.Status.RUNNING

    def make_command(self, uuid=None, enable_wait=False):
        """
        Export this policy execution as a complex action client command dictionary.
        """
        # Encode policy execution for MoveBlend composition.
        return self._make_command(
            "moveByPolicy", self.action_goal, uuid=uuid, enable_wait=enable_wait,
        )


def create_subtree(action_client, step_goal, **kwargs):
    """
    Create a policy execution subtree for one grounding step.

    An MoveBlend rather than a plain Sequence, and not for its own sake --
    with one child there is nothing here to overlap. It is so this root answers
    `current_progress()` and `overlappable` for the sequence that chains the
    steps: that is what lets a primitive start blending into this policy step,
    and this step into the primitive after it. Inert until the tree's
    `overlap_progress_threshold` drops below 1.0.

    Args:
        action_client (:class:`~rclpy.client.Client`): robot command client.
        step_goal (:obj:`dict`): policy command payload.

    Returns:
       :class:`~py_trees.behaviour.Behaviour`: subtree root
    """
    root = MoveBlend(
        name="Policy",
        threshold_param=OVERLAP_THRESHOLD_PARAM,
        dispatch_param=OVERLAP_DISPATCH_ON_START_PARAM,
        progress_threshold=1.0,
    )
    run_policy = MOVEBYPOLICY(
        name="MoveByPolicy",
        action_client=action_client,
        action_goal=step_goal,
        timeout=float(step_goal.get("timeout", 5.0)),
        robot_name=kwargs.get("robot_name"),
    )
    root.add_child(run_policy)
    return root
