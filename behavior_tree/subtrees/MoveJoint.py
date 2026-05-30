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
    def __init__(self, name, action_client, action_goal=None, timeout=3.0, robot_name=None):
        super(MOVEJ, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal,
                                   timeout=timeout,
                                   robot_name=robot_name)
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
                                  'timeout': self.timeout,
                                  'enable_wait': False})
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        self.feedback_message = "running"

        if self.current_goal_id() is None:
            return py_trees.common.Status.RUNNING
        
        if self.goal_matches_blackboard() and \
           self.current_goal_status() in [GoalStatus.STATUS_ABORTED,
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

        if self.goal_matches_blackboard() and \
           self.current_goal_status() is GoalStatus.STATUS_SUCCEEDED:
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

    def __init__(self, name, action_client, action_goal=None, robot_name=None):
        super(MOVEJR, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal,
                                   robot_name=robot_name)

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

        if self.current_goal_id() is None:
            return py_trees.common.Status.RUNNING
            
        if self.goal_matches_blackboard() and \
           self.current_goal_status() in [GoalStatus.STATUS_ABORTED,
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

        if self.goal_matches_blackboard() and \
           self.current_goal_status() is GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class MOVEJT(Move.MOVE):
    """
    Move through desired joint angle waypoints as one trajectory.

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None, timeout=3.0, robot_name=None):
        super(MOVEJT, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal,
                                   timeout=timeout,
                                   robot_name=robot_name)
        self.logger.debug("%s.__init__()" % self.__class__.__name__)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Fail when no command service client is available.
        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        # Send all joint waypoints as one trajectory command.
        if not self.sent_goal:
            goal = {}
            for i, waypoint in enumerate(self.action_goal):
                goal[str(i)] = waypoint

            # Create the uuid used to match status feedback.
            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)

            # Dispatch one moveJointTrajectory action to avoid stop-and-go MOVEJ chaining.
            cmd_str = json.dumps({'action_type': 'moveJointTrajectory',
                                  'goal': json.dumps(goal),
                                  'uuid': self.goal_uuid_des.tolist(),
                                  'timeout': self.timeout,
                                  'enable_wait': False})
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)

            self.sent_goal = True
            self.feedback_message = "Sending a joint trajectory goal"
            return py_trees.common.Status.RUNNING

        self.feedback_message = "running"

        # Wait until goal status is published to the blackboard.
        if self.current_goal_id() is None:
            return py_trees.common.Status.RUNNING

        # Treat terminal cancel/abort states as behaviour failure.
        if self.goal_matches_blackboard() and \
           self.current_goal_status() in [GoalStatus.STATUS_ABORTED,
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

        # Treat successful arm status as behaviour success.
        if self.goal_matches_blackboard() and \
           self.current_goal_status() is GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
                
