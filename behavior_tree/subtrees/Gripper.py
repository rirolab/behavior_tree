import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
from . import Move

from riro_srvs.srv import StringGoalStatus

class GOTO(Move.MOVE):
    """
    Move a gripper to the desired configuration

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """
    def __init__(self, name, action_client, action_goal=None,
                     force=1., check_contact=False, timeout=5):
        super(GOTO, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal)

        self.force         = force
        self.check_contact = check_contact
        self.timeout       = timeout


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)            
            cmd_str = json.dumps({'action_type': 'gripperGotoPos',
                                  'goal': self.action_goal,
                                  'uuid': self.goal_uuid_des.tolist(),
                                  'force': self.force,
                                  'check_contact': self.check_contact,
                                  'timeout': self.timeout,
                                  'enable_wait': True })
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a gripper goal"
            return py_trees.common.Status.RUNNING

        if self.blackboard.goal_id is None:
            return py_trees.common.Status.RUNNING
            
        if (self.goal_uuid_des == self.blackboard.goal_id).all() and \
           self.blackboard.goal_status in [
                                GoalStatus.STATUS_UNKNOWN,
                                ]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                  self.feedback_message))
            return py_trees.common.Status.FAILURE

        if (self.goal_uuid_des == self.blackboard.goal_id).all() and \
           self.blackboard.goal_status in [GoalStatus.STATUS_ABORTED,
                                               GoalStatus.STATUS_SUCCEEDED,
                                               GoalStatus.STATUS_CANCELING,
                                               GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
