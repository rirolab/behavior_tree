import numpy as np
import json
import rospy

import py_trees
import std_msgs.msg as std_msgs
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryResult
import geometry_msgs

from complex_action_client.srv import String_Int, None_String

class MOVEP(py_trees.behaviour.Behaviour):
    """
    Move Pose
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None,
                 topic_name="", controller_ns=""):
        super(MOVEP, self).__init__(name=name)

        self.topic_name = topic_name
        self.controller_ns = controller_ns
        self.action_goal = action_goal
        self.sent_goal   = False
        self.cmd_req     = None


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("arm_client/command")
        self.cmd_req = rospy.ServiceProxy("arm_client/command", String_Int)
        rospy.wait_for_service("arm_client/status")
        self.status_req = rospy.ServiceProxy("arm_client/status", None_String)    
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            if type(self.action_goal['pose']) is geometry_msgs.msg._Pose.Pose:
                goal = {'x': self.action_goal['pose'].position.x,
                        'y': self.action_goal['pose'].position.y,
                        'z': self.action_goal['pose'].position.z,
                        'qx': self.action_goal['pose'].orientation.x,
                        'qy': self.action_goal['pose'].orientation.y,
                        'qz': self.action_goal['pose'].orientation.z,
                        'qw': self.action_goal['pose'].orientation.w,}
            else:
                blackboard = py_trees.Blackboard()
                ps = blackboard.get(self.action_goal['pose'])
                goal = {'x': ps.position.x,
                        'y': ps.position.y,
                        'z': ps.position.z,
                        'qx': ps.orientation.x,
                        'qy': ps.orientation.y,
                        'qz': ps.orientation.z,
                        'qw': ps.orientation.w,}
            
            cmd_str = json.dumps({'action_type': 'movePose',
                                  'goal': json.dumps(goal),
                                  'timeout': 3.,
                                  'no_wait': True})

            ret = self.cmd_req(cmd_str)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                return py_trees.common.Status.FAILURE
            
            self.sent_goal        = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        ret   = d['result']

        if  state in [GoalStatus.ABORTED,
                      GoalStatus.PREEMPTED,
                      GoalStatus.REJECTED] and \
                      ret != FollowJointTrajectoryResult.SUCCESSFUL: 
            self.feedback_message = "FAILURE"
            return py_trees.common.Status.FAILURE

        if ret == FollowJointTrajectoryResult.SUCCESSFUL:
            self.feedback_message = "SUCCESSFUL"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        return


class MOVES(py_trees.behaviour.Behaviour):
    """
    Move Pose following a straight pose trajectory
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None,
                 topic_name="", controller_ns=""):
        super(MOVES, self).__init__(name=name)

        self.topic_name = topic_name
        self.controller_ns = controller_ns
        self.action_goal = action_goal
        self.sent_goal   = False
        self.cmd_req     = None


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("arm_client/command")
        self.cmd_req = rospy.ServiceProxy("arm_client/command", String_Int)
        rospy.wait_for_service("arm_client/status")
        self.status_req = rospy.ServiceProxy("arm_client/status", None_String)    
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            # reference frame: arm_baselink
            if type(self.action_goal['pose']) is geometry_msgs.msg._Pose.Pose:
                goal = {'x': self.action_goal['pose'].position.x,
                        'y': self.action_goal['pose'].position.y,
                        'z': self.action_goal['pose'].position.z,
                        'qx': self.action_goal['pose'].orientation.x,
                        'qy': self.action_goal['pose'].orientation.y,
                        'qz': self.action_goal['pose'].orientation.z,
                        'qw': self.action_goal['pose'].orientation.w,}
            else:
                blackboard = py_trees.Blackboard()
                ps = blackboard.get(self.action_goal['pose'])                
                goal = {'x': ps.position.x,
                        'y': ps.position.y,
                        'z': ps.position.z,
                        'qx': ps.orientation.x,
                        'qy': ps.orientation.y,
                        'qz': ps.orientation.z,
                        'qw': ps.orientation.w,}
            
            cmd_str = json.dumps({'action_type': 'movePoseStraight',
                                  'goal': json.dumps(goal),
                                  'timeout': 3.,
                                  'no_wait': True})
            ret = self.cmd_req(cmd_str)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                return py_trees.common.Status.FAILURE
            
            self.sent_goal        = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        ret   = d['result']
        
        if  state in [GoalStatus.ABORTED,
                      GoalStatus.PREEMPTED,
                      GoalStatus.REJECTED] and \
                      ret != FollowJointTrajectoryResult.SUCCESSFUL: 
            self.feedback_message = "FAILURE"
            return py_trees.common.Status.FAILURE

        if ret == FollowJointTrajectoryResult.SUCCESSFUL:
            self.feedback_message = "SUCCESSFUL"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        return



class MOVEPR(py_trees.behaviour.Behaviour):
    """
    Move Pose Relative with a certain frame
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None,
                 topic_name="", controller_ns="", cont=False):
        super(MOVEPR, self).__init__(name=name)

        self.topic_name = topic_name
        self.controller_ns = controller_ns
        
        # Set the goal pose
        self.action_goal = action_goal
        # Enable continuous motion
        self.action_cont = cont
        # Set the goal flag
        self.sent_goal = False
        self.cmd_req   = None


    def setup(self, timeout):
        ## self.publisher = rospy.Publisher(self.topic_name, std_msgs.String, queue_size=10, latch=True)
        self.feedback_message = "{}: setup".format(self.name)
        
        rospy.wait_for_service("arm_client/command")
        self.cmd_req = rospy.ServiceProxy("arm_client/command", String_Int)
        rospy.wait_for_service("arm_client/status")
        self.status_req = rospy.ServiceProxy("arm_client/status", None_String)

        if self.action_cont:
            timeout_scale = 0.5
        else:
            timeout_scale = 1.
        self.cmd_req(json.dumps({'action_type': 'setSpeed', 'goal': timeout_scale}))        
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


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
            
            cmd_str = json.dumps({'action_type': 'movePoseRelative',
                                  'goal': json.dumps(goal),
                                  'frame': self.action_goal['frame'],
                                  'timeout': 3.,
                                  'no_wait': True})
            ret = self.cmd_req(cmd_str)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = \
                  "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                return py_trees.common.Status.FAILURE
                        
            self.sent_goal = True
            self.feedback_message = "Sending a pose goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        ret   = d['result']
        
        if  state in [GoalStatus.ABORTED,
                      GoalStatus.PREEMPTED,
                      GoalStatus.REJECTED] and \
                      ret != FollowJointTrajectoryResult.SUCCESSFUL: 
            self.feedback_message = "FAILURE"
            return py_trees.common.Status.FAILURE

        if ret == FollowJointTrajectoryResult.SUCCESSFUL:
            self.feedback_message = "SUCCESSFUL"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        return


class MOVEPROOT(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None,
                 topic_name="", controller_ns=""):
        super(MOVEPROOT, self).__init__(name=name)

        self.topic_name = topic_name
        self.controller_ns = controller_ns
        self.action_goal = action_goal
        self.sent_goal = False
        self.cmd_req   = None

    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("arm_client/command")
        self.cmd_req = rospy.ServiceProxy("arm_client/command", String_Int)
        rospy.wait_for_service("arm_client/status")
        self.status_req = rospy.ServiceProxy("arm_client/status", None_String)
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            # reference frame: arm_baselink
            if type(self.action_goal['pose']) is geometry_msgs.msg._Pose.Pose:
                goal = {'x': self.action_goal['pose'].position.x,
                        'y': self.action_goal['pose'].position.y,
                        'z': self.action_goal['pose'].position.z,
                        'qx': self.action_goal['pose'].orientation.x,
                        'qy': self.action_goal['pose'].orientation.y,
                        'qz': self.action_goal['pose'].orientation.z,
                        'qw': self.action_goal['pose'].orientation.w,}
            else:
                blackboard = py_trees.Blackboard()
                ps = blackboard.get(self.action_goal['pose'])                
                goal = {'x': ps.position.x,
                        'y': ps.position.y,
                        'z': ps.position.z,
                        'qx': ps.orientation.x,
                        'qy': ps.orientation.y,
                        'qz': ps.orientation.z,
                        'qw': ps.orientation.w,}
            
            cmd_str = json.dumps({'action_type': 'movePoseRoot',
                                  'goal': json.dumps(goal),
                                  'timeout': 3.,
                                  'no_wait': True})
            ret = self.cmd_req(cmd_str)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                return py_trees.common.Status.FAILURE
            
            self.sent_goal        = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        ret   = d['result']
        # print("#######\n", d, "\n#####")
        # print("#######\n", state, "\n#####")
        # print("#######\n", ret, "\n#####")
        
        if  state in [GoalStatus.ABORTED,
                      GoalStatus.PREEMPTED,
                      GoalStatus.REJECTED] and \
                      ret != FollowJointTrajectoryResult.SUCCESSFUL: 
            self.feedback_message = "FAILURE"
            return py_trees.common.Status.FAILURE

        if ret == FollowJointTrajectoryResult.SUCCESSFUL:
            self.feedback_message = "SUCCESSFUL"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
            
    

    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))            
        return 
