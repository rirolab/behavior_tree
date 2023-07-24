import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
from . import Move

from riro_srvs.srv import StringGoalStatus

class STOP(Move.MOVE):
    """
    Stop the current movement
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """
    def __init__(self, name, action_client, action_goal=None):
        super(STOP, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal)
        
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            cmd_str = json.dumps({'action_type': 'cancel_goal',
                                  'enable_wait': False})

            req = StringGoalStatus.Request()
            req.data = cmd_str
            
            self.future = self.cmd_req.call_async(req)
            self.sent_goal = True
            self.feedback_message = "Cancelling a goal"
            return py_trees.common.Status.RUNNING


        if self.blackboard.goal_status is GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                    py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    
    def terminate(self, new_status):
        """ """
        self.logger.debug("%s.terminate()" % self.__class__.__name__)

        if self.blackboard.goal_status == GoalStatus.STATUS_EXECUTING:
            req = StringGoalStatus.Request()
            req.data = json.dumps({'action_type': 'cancel_goal',
                                   'enable_wait': True})
            self.future = self.cmd_req.call_async( req )
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))            
        return py_trees.common.Status.SUCCESS
    
