import json

import numpy as np
import py_trees
from action_msgs.msg import GoalStatus

from riro_srvs.srv import StringGoalStatus

from . import Move


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
        self.logger.debug("%s.__init__()" % self.__class__.__name__)

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


def create_subtree(action_client, step_goal, **kwargs):
    """
    Create a policy execution subtree for one grounding step.

    Args:
        action_client (:class:`~rclpy.client.Client`): robot command client.
        step_goal (:obj:`dict`): policy command payload.

    Returns:
       :class:`~py_trees.behaviour.Behaviour`: subtree root
    """
    root = py_trees.composites.Sequence(name="Policy", memory=True)
    run_policy = MOVEBYPOLICY(
        name="MoveByPolicy",
        action_client=action_client,
        action_goal=step_goal,
        timeout=float(step_goal.get("timeout", 5.0)),
        robot_name=kwargs.get("robot_name"),
    )
    root.add_child(run_policy)
    return root
