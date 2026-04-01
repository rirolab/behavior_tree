import json

import numpy as np
import py_trees
from action_msgs.msg import GoalStatus

from riro_srvs.srv import StringGoalStatus

from . import Move


class RUN(Move.MOVE):
    """
    Execute policy through complex_action_client.
    """

    def __init__(self, name, action_client, action_goal=None, timeout=5.0):
        super(RUN, self).__init__(
            name=name,
            action_client=action_client,
            action_goal=action_goal,
            timeout=timeout,
        )
        self.logger.debug("%s.__init__()" % self.__class__.__name__)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16, dtype=np.uint8)
            cmd_str = json.dumps(
                {
                    "action_type": "run_policy",
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

        if self.blackboard.goal_id is None:
            return py_trees.common.Status.RUNNING

        if not (self.goal_uuid_des == self.blackboard.goal_id).all():
            return py_trees.common.Status.RUNNING

        status = self.blackboard.goal_status
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
