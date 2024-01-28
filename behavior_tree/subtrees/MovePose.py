import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
from . import Move

from riro_srvs.srv import StringGoalStatus
import geometry_msgs
import py_trees.console as console

## import std_msgs.msg as std_msgs
## from actionlib_msgs.msg import GoalStatus
## from control_msgs.msg import FollowJointTrajectoryResult

## from complex_action_client.srv import String_Int, None_String

class MOVEP(Move.MOVE):
    """
    Move to a Cartesian pose
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None):
        super(MOVEP, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal)

        self.blackboard.register_key(key=self.action_goal['pose'], \
                                     access=py_trees.common.Access.READ)
        self.name = name

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            if type(self.action_goal['pose']) is geometry_msgs.msg.Pose:
                goal = {'x': self.action_goal['pose'].position.x,
                        'y': self.action_goal['pose'].position.y,
                        'z': self.action_goal['pose'].position.z,
                        'qx': self.action_goal['pose'].orientation.x,
                        'qy': self.action_goal['pose'].orientation.y,
                        'qz': self.action_goal['pose'].orientation.z,
                        'qw': self.action_goal['pose'].orientation.w,}
            else:
                ps = self.blackboard.get(self.action_goal['pose'])
                goal = {'x': ps.position.x,
                        'y': ps.position.y,
                        'z': ps.position.z,
                        'qx': ps.orientation.x,
                        'qy': ps.orientation.y,
                        'qz': ps.orientation.z,
                        'qw': ps.orientation.w,}
                # print("MOVEPOSE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n", self.name, goal)
            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)
                    
            cmd_str = json.dumps({'action_type': 'movePose',
                                  'goal': json.dumps(goal),
                                  'uuid': self.goal_uuid_des.tolist(),
                                  'timeout': 3.,
                                  'enable_wait': False})
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a pose goal"
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


        
class MOVES(Move.MOVE):
    """
    Move to a Cartesian pose following a straight pose trajectory
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None):
        super(MOVES, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal)

        self.blackboard.register_key(key=self.action_goal['pose'], \
                                     access=py_trees.common.Access.READ)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            # reference frame: arm_baselink
            if type(self.action_goal['pose']) is geometry_msgs.msg.Pose:
                goal = {'x': self.action_goal['pose'].position.x,
                        'y': self.action_goal['pose'].position.y,
                        'z': self.action_goal['pose'].position.z,
                        'qx': self.action_goal['pose'].orientation.x,
                        'qy': self.action_goal['pose'].orientation.y,
                        'qz': self.action_goal['pose'].orientation.z,
                        'qw': self.action_goal['pose'].orientation.w,}
            else:
                ps = self.blackboard.get(self.action_goal['pose'])                
                goal = {'x': ps.position.x,
                        'y': ps.position.y,
                        'z': ps.position.z,
                        'qx': ps.orientation.x,
                        'qy': ps.orientation.y,
                        'qz': ps.orientation.z,
                        'qw': ps.orientation.w,}
            
            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)
            cmd_str = json.dumps({'action_type': 'movePoseStraight',
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



class MOVEPR(Move.MOVE):
    """
    Move Pose Relative with a certain frame
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """
    def __init__(self, name, action_client, action_goal=None,\
                     cont=False):
        super(MOVEPR, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal)

        self.blackboard.register_key(key=self.action_goal['pose'], \
                                     access=py_trees.common.Access.READ)
                                     
        # Enable continuous motion
        self.action_cont = cont


    def setup(self, timeout):

        super.setup(timeout)

        if self.action_cont:
            timeout_scale = 0.5
        else:
            timeout_scale = 1.
        self.cmd_req(json.dumps({'action_type': 'setSpeed', 'goal': timeout_scale}))        
        return True


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal or (self.action_cont and self.action_goal['pose'] is not None):
            goal = {'x': self.action_goal['pose'].position.x,
                    'y': self.action_goal['pose'].position.y,
                    'z': self.action_goal['pose'].position.z,
                    'qx': self.action_goal['pose'].orientation.x,
                    'qy': self.action_goal['pose'].orientation.y,
                    'qz': self.action_goal['pose'].orientation.z,
                    'qw': self.action_goal['pose'].orientation.w,}                    

            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)
                
            cmd_str = json.dumps({'action_type': 'movePoseRelative',
                                  'goal': json.dumps(goal),
                                  'frame': self.action_goal['frame'],
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


class MOVEPROOT(Move.MOVE):
    """
    Move the root joint of the manipulator toward pick-and-place of a target object (pose).

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None):
        super(MOVEPROOT, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal)
    
        self.blackboard.register_key(key=self.action_goal['pose'], \
                                     access=py_trees.common.Access.READ)
                                     
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            # reference frame: arm_baselink
            if type(self.action_goal['pose']) is geometry_msgs.msg.Pose:
                goal = {'x': self.action_goal['pose'].position.x,
                        'y': self.action_goal['pose'].position.y,
                        'z': self.action_goal['pose'].position.z,
                        'qx': self.action_goal['pose'].orientation.x,
                        'qy': self.action_goal['pose'].orientation.y,
                        'qz': self.action_goal['pose'].orientation.z,
                        'qw': self.action_goal['pose'].orientation.w,}
            else:
                ps = self.blackboard.get(self.action_goal['pose'])                
                goal = {'x': ps.position.x,
                        'y': ps.position.y,
                        'z': ps.position.z,
                        'qx': ps.orientation.x,
                        'qy': ps.orientation.y,
                        'qz': ps.orientation.z,
                        'qw': ps.orientation.w,}

            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)
                    
            cmd_str = json.dumps({'action_type': 'movePoseRoot',
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
            
