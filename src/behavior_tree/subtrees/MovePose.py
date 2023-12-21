import numpy as np
import json
import rospy

import py_trees
import std_msgs.msg as std_msgs
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryResult
import geometry_msgs

from complex_action_client.srv import String_Int, None_String

import tf

from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse

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
        
        self._manip_status_update_srv_channel = "/update_robot_manip_state"



    def setup(self, timeout):
        rospy.loginfo(f"[Subtree] MOVEP : setup() called ({self.name}).")
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("arm_client/command")
        self.cmd_req = rospy.ServiceProxy("arm_client/command", String_Int)
        rospy.wait_for_service("arm_client/status")
        self.status_req = rospy.ServiceProxy("arm_client/status", None_String)    

        rospy.wait_for_service(self._manip_status_update_srv_channel)
        self.manip_status_update_req = rospy.ServiceProxy(self._manip_status_update_srv_channel, String_None)
        rospy.loginfo(f"[Subtree] MOVEP : setup() done ({self.name}).")
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        blackboard = py_trees.Blackboard()
        self.manip_status_update_req(blackboard.robot_name)

    def update(self):

        self.logger.debug("%s.update()" % self.__class__.__name__)
        blackboard = py_trees.Blackboard()
        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            if type(self.action_goal['pose']) is geometry_msgs.msg._Pose.Pose:
                # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
                # exit()
                goal = {'x': self.action_goal['pose'].position.x,
                        'y': self.action_goal['pose'].position.y,
                        'z': self.action_goal['pose'].position.z,
                        'qx': self.action_goal['pose'].orientation.x,
                        'qy': self.action_goal['pose'].orientation.y,
                        'qz': self.action_goal['pose'].orientation.z,
                        'qw': self.action_goal['pose'].orientation.w,}
            else:
                # print("!22222!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n") ## through here
                # exit()
                blackboard = py_trees.Blackboard()
                ps = blackboard.get(self.action_goal['pose'])
                goal = {'x': ps.position.x,
                        'y': ps.position.y,
                        'z': ps.position.z,
                        'qx': ps.orientation.x,
                        'qy': ps.orientation.y,
                        'qz': ps.orientation.z,
                        'qw': ps.orientation.w,}
                # print("fcvqefeqfqefqeqewfeqwfeqw!!!!!!!!!!!!!!!!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n", goal, self.action_goal['pose'])
                # raise NotImplementedError


                br = tf.TransformBroadcaster()
                br.sendTransform((ps.position.x, ps.position.y, ps.position.z), (ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w), rospy.Time.now(), self.action_goal['pose'] + "_mj", 'haetae_ur5e_base_link')
                
            cmd_str = json.dumps({'action_type': 'movePose',
                                  'goal': json.dumps(goal),
                                  'timeout': 3.,
                                  'no_wait': True})

            ret = self.cmd_req(cmd_str)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                # self.manip_status_update_req(blackboard.robot_name)
                return py_trees.common.Status.FAILURE
            
            self.sent_goal        = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        # print("!!!!#!@$#!@$#!@$#@!$!@!QR#@!$!#@$#!$!\n\n\n")
        # exit()

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        ret   = d['result']
        rospy.loginfo(f"[SubTree] MOVEP : state = {state}, ret = {ret}  <---------------------------")
        terminal_states = [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED]
        if  state in [GoalStatus.ABORTED,
                      GoalStatus.PREEMPTED,
                      GoalStatus.REJECTED] and \
                      ret != FollowJointTrajectoryResult.SUCCESSFUL: 
            self.feedback_message = "FAILURE"
            self.manip_status_update_req(blackboard.robot_name)
            return py_trees.common.Status.FAILURE

        if state in terminal_states and \
                    ret == FollowJointTrajectoryResult.SUCCESSFUL:
            self.feedback_message = "SUCCESSFUL"
            self.manip_status_update_req(blackboard.robot_name)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        
        blackboard = py_trees.Blackboard()
        self.manip_status_update_req(blackboard.robot_name)
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
        self._manip_status_update_srv_channel = "/update_robot_manip_state"

    def setup(self, timeout):
        rospy.loginfo("[Subtree] MOVEPROOT : setup() called.")
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("arm_client/command")
        self.cmd_req = rospy.ServiceProxy("arm_client/command", String_Int)
        rospy.wait_for_service("arm_client/status")
        self.status_req = rospy.ServiceProxy("arm_client/status", None_String)
        rospy.loginfo("[Subtree] MOVEPROOT : setup() done.")
        rospy.wait_for_service(self._manip_status_update_srv_channel)
        self.manip_status_update_req = rospy.ServiceProxy(self._manip_status_update_srv_channel, String_None)

        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        blackboard = py_trees.Blackboard()
        self.manip_status_update_req(blackboard.robot_name)


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        blackboard = py_trees.Blackboard()
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
            
            cmd_str = json.dumps({'action_type' : 'movePoseRoot',
                                  'goal'        : json.dumps(goal),
                                  'timeout'     : 3.,
                                  'no_wait'     : True})
            ret = self.cmd_req(cmd_str)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                # self.manip_status_update_req(blackboard.robot_name)
                return py_trees.common.Status.FAILURE
            
            self.sent_goal        = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        ret   = d['result']
        
        rospy.loginfo(f"[SubTree] MOVEPROOT : state = {state}, ret = {ret}  <---------------------------")
        terminal_states = [GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED]
        
        # print("#######\n", d, "\n#####")
        # print("#######\n", state, "\n#####")
        # print("#######\n", ret, "\n#####")
        
        if  state in [GoalStatus.ABORTED,
                      GoalStatus.PREEMPTED,
                      GoalStatus.REJECTED] and \
                      ret != FollowJointTrajectoryResult.SUCCESSFUL: 
            self.feedback_message = "FAILURE"
            self.manip_status_update_req(blackboard.robot_name)
            return py_trees.common.Status.FAILURE

        if state in terminal_states and ret == FollowJointTrajectoryResult.SUCCESSFUL:
            self.feedback_message = "SUCCESSFUL"
            self.manip_status_update_req(blackboard.robot_name)
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
            
    

    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))    

        blackboard = py_trees.Blackboard()
        self.manip_status_update_req(blackboard.robot_name)        
        return 
