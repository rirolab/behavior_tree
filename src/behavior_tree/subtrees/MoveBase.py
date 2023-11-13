import numpy as np
import json
import rospy
import actionlib
import py_trees
import sys
import tf
import threading
from copy import deepcopy

import std_msgs.msg as std_msgs
import geometry_msgs
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult, MoveBaseGoal
from complex_action_client import misc
from complex_action_client.srv import String_Int, None_String, String_IntRequest

from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse

class MOVEB(py_trees.behaviour.Behaviour):
    """
    Move Base
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None):
        super(MOVEB, self).__init__(name=name)

        # self.topic_name = topic_name
        # self.controller_ns = controller_ns
        self.action_goal = action_goal
        self.sent_goal   = False
        self._world_frame_id = rospy.get_param("/world_frame", 'map')
        self.cmd_req     = None

        self._drive_status_update_srv_channel = "/update_robot_drive_state"


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("move_base_client/command")
        self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
        rospy.wait_for_service("move_base_client/status")
        self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)

        rospy.wait_for_service(self._drive_status_update_srv_channel)
        self.drive_status_update_req = rospy.ServiceProxy(self._drive_status_update_srv_channel, String_None)

        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        blackboard = py_trees.Blackboard()
        self.drive_status_update_req(blackboard.robot_name)

    def update(self):

        blackboard = py_trees.Blackboard()
        # listener = tf.TransformListener()
        # self.drive_status_update_req(blackboard.robot_name)
        # if blackboard.robot_name == 'spot':
        #     while not rospy.is_shutdown():
        #         try:
        #             (pos, quat) = listener.lookupTransform("spot/base_link_plate", "box_s_grip_1", rospy.Time(0))
        #             (pos2, quat2) = listener.lookupTransform("spot/base_link", "picking_station2_nav_1", rospy.Time(0))
                    
        #             print("!!!!!!!!!!\n\n", pos, len(pos))
        #             print("!!",blackboard.robot_name)
        #             # assert len(pos) == 3
        #             box_plate_dist = pos[0] ** 2 + pos[1] ** 2 + pos[2] ** 2
        #             spot_to_ps2 = pos2[0] ** 2 + pos2[1] ** 2 + pos2[2] ** 2
        #             print(box_plate_dist, spot_to_ps2)
        #             if box_plate_dist > 1.1: ## from origin -> picking_station2
        #                 break
        #             elif box_plate_dist < 0.5 and spot_to_ps2 < 0.7: ## stay at picking_station2
        #                 break
        #         except:
        #             print("111111!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!23232\n")
        #             pass

        # elif blackboard.robot_name == 'haetae':
        #     pass
        
        # elif blackboard.robot_name == 'stretch':
        #     pass

        # else:
        #     raise NotImplementedError()


        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "No action client, did you call setup() on your tree?"
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

            cmd_str = json.dumps({'action_type': 'moveBase',
                                  'frame': self._world_frame_id,
                                  'goal': json.dumps(goal),
                                  'timeout': 10.,
                                  'no_wait': True})

            ret = self.cmd_req(cmd_str)
            print("(MOVEBASE update) goal: ", goal)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                self.drive_status_update_req(blackboard.robot_name)
                return py_trees.common.Status.FAILURE
            
            self.sent_goal        = True
            self.feedback_message = "Sending a navigation goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        
        if state in [GoalStatus.ABORTED, GoalStatus.PREEMPTED, GoalStatus.REJECTED]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.FAILURE, self.feedback_message))
            self.drive_status_update_req(blackboard.robot_name)
            return py_trees.common.Status.FAILURE

        if state == GoalStatus.SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.SUCCESS, self.feedback_message))
            
            self.drive_status_update_req(blackboard.robot_name)
            
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        return


class MOVEBR(py_trees.behaviour.Behaviour):
    """
    Move Base Relative to Base_Link
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None):
        super(MOVEBR, self).__init__(name=name)

        self.action_goal = action_goal
        self.sent_goal   = False
        self._frame_id = rospy.get_param("torso_frame", 'base_link')
        self.cmd_req     = None


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("move_base_client/command")
        self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
        rospy.wait_for_service("move_base_client/status")
        self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "No action client, did you call setup() on your tree?"
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

            cmd_str = json.dumps({'action_type': 'moveBase',
                                  'frame': self._frame_id,
                                  'goal': json.dumps(goal),
                                  'timeout': 10.,
                                  'no_wait': True})

            ret = self.cmd_req(cmd_str)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                return py_trees.common.Status.FAILURE
            
            self.sent_goal        = True
            self.feedback_message = "Sending a navigation goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        
        if state in [GoalStatus.ABORTED, GoalStatus.PREEMPTED, GoalStatus.REJECTED]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.FAILURE, self.feedback_message))
            return py_trees.common.Status.FAILURE

        if state == GoalStatus.SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.SUCCESS, self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        return


class ALIGNB(py_trees.behaviour.Behaviour):
    """
    Rotate Base to align with goal pose 
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None):
        super(ALIGNB, self).__init__(name=name)

        self.action_goal = action_goal
        # self.sent_goal   = False
        self._world_frame_id = rospy.get_param("/world_frame", 'map')
        self._base_frame_id = rospy.get_param("torso_frame", "base_link")
        self.cmd_req     = None


    def setup(self, timeout):
        self.listener = tf.TransformListener()
        self.feedback_message = "{}: setup".format(self.name)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        # rospy.wait_for_service("move_base_client/command")
        # self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
        # rospy.wait_for_service("move_base_client/status")
        # self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        curr_quat = None
        while not rospy.is_shutdown():
            try:
                (curr_pos,curr_quat) = self.listener.lookupTransform(self._world_frame_id, self._base_frame_id, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.feedback_message = "ALIGNBASE: Exception from TF"
                continue
            if curr_quat is not None: break
        curr_ps = misc.list2KDLframe(curr_pos+curr_quat)
        
        if type(self.action_goal['pose']) is geometry_msgs.msg._Pose.Pose:
            target_ps = self.action_goal['pose']
        else:
            blackboard = py_trees.Blackboard()
            target_ps = blackboard.get(self.action_goal['pose'])
        target_ps = misc.pose2KDLframe(target_ps)
        yaw_diff = (curr_ps.Inverse() * target_ps).M.GetRPY()[2]
        
        cmd_vel = geometry_msgs.msg.Twist()
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0.0
        from math import copysign
        if abs(yaw_diff) < 0.05:
            cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd_vel)
            self.feedback_message = "Completed a alignment goal"
            return py_trees.common.Status.SUCCESS
        else:
            cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = copysign(0.1, yaw_diff)
            self.cmd_vel_pub.publish(cmd_vel)

            self.feedback_message = "Rotating the base."
            return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        # msg = self.status_req()
        # d = json.loads(msg.data)
        # if d['state'] == GoalStatus.ACTIVE:
        #     self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        return

class TOUCHB(py_trees.behaviour.Behaviour):
    """
    Rotate Base to align with goal pose 
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name):
        super(TOUCHB, self).__init__(name=name)

        self.scan_topic   = "front_rgbd_camera/scan"
        self.scan = None
        self.steps = 0
        self._lock = threading.Lock()

    def setup(self, timeout):
        self.scan_sub = rospy.Subscriber(self.scan_topic, LaserScan, self._laserscan_cb)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", geometry_msgs.msg.Twist, queue_size=1)
        self.feedback_message = "{}: setup".format(self.name)
        return True

    def _laserscan_cb(self, msg):
        with self._lock:
            self.scan = deepcopy(msg)

    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.scan = None
        self.steps = 0

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        with self._lock:
            scan = deepcopy(self.scan)
        # if scan is None:
        #     self.feedback_message = "(Touch Base) scan topic not subscribed."
        #     return py_trees.common.Status.RUNNING
        
        cmd_vel = geometry_msgs.msg.Twist()
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0.0
        if self.steps > 6:
            print("[TOUCH] Touch Done.")
            self.cmd_vel_pub.publish(cmd_vel)        
            return py_trees.common.Status.SUCCESS
        else:
            print("[TOuch] ", self.steps)
            cmd_vel.linear.x = 0.1
            self.cmd_vel_pub.publish(cmd_vel)        
            self.steps += 1
            return py_trees.common.Status.RUNNING
        
        
        # ranges = np.array(scan.ranges)
        
        # valid_index = np.where((ranges < scan.range_max))[0]
        # print("(len of vaild index) ", len(valid_index))
        # valid_ranges = ranges[valid_index]
        # if len(valid_index) == 0:
        #     cmd_vel = geometry_msgs.msg.Twist()
        #     cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0.0
        #     cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0.0
        #     self.cmd_vel_pub.publish(cmd_vel)        
        #     return py_trees.common.Status.SUCCESS
        
        # projected_ranges = scan.angle_min + scan.angle_increment*valid_index
        # projected_ranges = np.cos(projected_ranges) * valid_ranges
        # min_dist = np.min(projected_ranges)
        # print("(closest obs) ", min_dist)        
        # cmd_vel = geometry_msgs.msg.Twist()
        # cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0.0
        # cmd_vel.angular.x = cmd_vel.angular.y = cmd_vel.angular.z = 0.0
        
        # if min_dist < (scan.range_min+0.05):
        #     self.cmd_vel_pub.publish(cmd_vel)
        #     self.feedback_message = "Completed a alignment goal"
        #     return py_trees.common.Status.SUCCESS
        # else:
        #     cmd_vel.linear.x = 0.01
        #     self.cmd_vel_pub.publish(cmd_vel)
        #     self.feedback_message = "(Touch Base) Move Forward."
        #     return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        # msg = self.status_req()
        # d = json.loads(msg.data)
        # if d['state'] == GoalStatus.ACTIVE:
        #     self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        return

