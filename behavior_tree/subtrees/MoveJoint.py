import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
from . import Move

from riro_srvs.srv import StringGoalStatus

class MOVEJ(Move.MOVE):
    """
    Move to the desired joint angles.

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """
    def __init__(self, name, action_client, action_goal=None):
        super(MOVEJ, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal)
        self.logger.debug("%s.__init__()" % self.__class__.__name__)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE
            
        if not self.sent_goal:
            goal = {}
            for i, ang in enumerate(self.action_goal):
                goal[str(i)] = ang

            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)
                
            cmd_str = json.dumps({'action_type': 'moveJoint',
                                  'goal': json.dumps(goal),
                                  'uuid': self.goal_uuid_des.tolist(),
                                  'timeout': 3.,
                                  'enable_wait': False})
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        self.feedback_message = "running"

        if self.blackboard.goal_id is None:
            return py_trees.common.Status.RUNNING
        
        if (self.goal_uuid_des == self.blackboard.goal_id).all() and \
           self.blackboard.goal_status in [GoalStatus.STATUS_ABORTED,
                                GoalStatus.STATUS_UNKNOWN,
                                GoalStatus.STATUS_CANCELING,
                                GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                  self.feedback_message))
            return py_trees.common.Status.FAILURE

        if (self.goal_uuid_des == self.blackboard.goal_id).all() and \
           self.blackboard.goal_status is GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
                

class MOVEJ_FIT(Move.MOVE):
    """
    Move to the desired joint angles. With given "intermediate_pose"

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """
    def __init__(self, name, action_client, action_goal=None):
        super(MOVEJ_FIT, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal)
        self.logger.debug("%s.__init__()" % self.__class__.__name__)
        
        self.blackboard.register_key(key='intermediate_pose', access=py_trees.common.Access.READ)
        # print("DFQFEQWFQFQ!!!!\n\n\n\n\n\n\n", self.blackboard.intermediate_pose)
        # self.action_goal = self.blackboard.intermediate_pose

    def initialise(self):
        self.action_goal = self.blackboard.intermediate_pose

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # print("DFQFEQWFQFQ!!!!\n\n\n\n\n\n\n", self.blackboard.intermediate_pose)
        # self.action_goal = self.blackboard.intermediate_pose

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE
            
        if not self.sent_goal:
            goal = {}
            for i, ang in enumerate(self.action_goal):
                goal[str(i)] = ang

            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)
                
            cmd_str = json.dumps({'action_type': 'moveJoint',
                                  'goal': json.dumps(goal),
                                  'uuid': self.goal_uuid_des.tolist(),
                                  'timeout': 3.,
                                  'enable_wait': False})
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        self.feedback_message = "running"

        if self.blackboard.goal_id is None:
            return py_trees.common.Status.RUNNING
        
        if (self.goal_uuid_des == self.blackboard.goal_id).all() and \
           self.blackboard.goal_status in [GoalStatus.STATUS_ABORTED,
                                GoalStatus.STATUS_UNKNOWN,
                                GoalStatus.STATUS_CANCELING,
                                GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                  self.feedback_message))
            return py_trees.common.Status.FAILURE

        if (self.goal_uuid_des == self.blackboard.goal_id).all() and \
           self.blackboard.goal_status is GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING



class MOVEJR(Move.MOVE):
    """
    Move a desired displacement of joint angles.

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None):
        super(MOVEA, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
            
        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            goal = {}
            for i, ang in enumerate(self.action_goal):
                goal[str(i)] = ang

            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)                
            cmd_str = json.dumps({'action_type': 'moveJointRelative',
                                  'goal': json.dumps(goal),
                                  'uuid': self.goal_uuid_des.tolist(),
                                  'timeout': 3.,
                                  'enable_wait': False})
            req = StringGoalStatus.Request(data=cmd_str)            
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        if self.blackboard.goal_id is None:
            return py_trees.common.Status.RUNNING
            
        if (self.goal_uuid_des == self.blackboard.goal_id).all() and \
           self.blackboard.goal_status in [GoalStatus.STATUS_ABORTED,
                                GoalStatus.STATUS_UNKNOWN,
                                GoalStatus.STATUS_CANCELING,
                                GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                  self.feedback_message))
            return py_trees.common.Status.FAILURE

        if (self.goal_uuid_des == self.blackboard.goal_id).all() and \
           self.blackboard.goal_status is GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
                

