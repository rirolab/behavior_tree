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
from geometry_msgs.msg import Quaternion, TwistWithCovarianceStamped, Twist, Pose
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult, MoveBaseGoal
from complex_action_client import misc
from complex_action_client.srv import String_Int, None_String, String_IntRequest

from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse, String_Dup_None, String_Dup_NoneRequest

# class MOVEB(py_trees.behaviour.Behaviour):
#     """
#     Move Base
    
#     Note that this behaviour will return with
#     :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
#     command to the robot if it is cancelled or interrupted by a higher
#     priority behaviour.
#     """

#     def __init__(self, name, action_goal=None, is_collab=False, destination=None):
#         super(MOVEB, self).__init__(name=name)

#         # self.topic_name = topic_name
#         # self.controller_ns = controller_ns
#         self.action_goal = action_goal
#         self.sent_goal   = False
#         self._world_frame_id = rospy.get_param("/world_frame", 'map')
#         self.cmd_req     = None

#         self._drive_status_update_srv_channel = "/update_robot_drive_state"

#         self._spot_cmd_vel = "/spot/cmd_vel"

#         self.is_collab = is_collab
#         if self.is_collab == True:
#             self._spot2haetae_cmd_vel = "sibal/cmd_vel"
#         else:
#             self._spot2haetae_cmd_vel = None

#         self.destination = destination
#         self._arrival_state_udpate_srv_channel = "/update_arrival_state"
#         self._arrival_state_delete_srv_channel = '/delete_arrival_state'

#     def setup(self, timeout):
#         self.feedback_message = "{}: setup".format(self.name)
#         rospy.wait_for_service("move_base_client/command")
#         self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
#         rospy.wait_for_service("move_base_client/status")
#         self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)

#         rospy.wait_for_service(self._drive_status_update_srv_channel)
#         self.drive_status_update_req = rospy.ServiceProxy(self._drive_status_update_srv_channel, String_None)

#         rospy.wait_for_service(self._arrival_state_udpate_srv_channel)
#         self.arrival_status_update_req = rospy.ServiceProxy(self._arrival_state_udpate_srv_channel, String_Dup_None)
#         # self.arrival_status_update_req = rospy.ServiceProxy(self._arrival_state_udpate_srv_channel, String_None)

#         rospy.wait_for_service(self._arrival_state_delete_srv_channel)
#         self.arrival_status_delete_req = rospy.ServiceProxy(self._arrival_state_delete_srv_channel, String_None)


#         if self.is_collab == True:
#             spot_init_pose = self.get_obj_pose_from_wm("spot")
#             haeate_init_pose = self.get_obj_pose_from_wm("haetae")
#             self.spot_haetae_x_diff = spot_init_pose[0] - haeate_init_pose[0]
#             self.spot_haetae_y_diff = spot_init_pose[1] - haeate_init_pose[1]

#         blackboard = py_trees.Blackboard()
#         self.spot_cmd_vel_sub = None
#         self.spot2haetae_cmd_vel_pub = None
#         if self.is_collab == True:
#             if blackboard.robot_name == "spot":
#                 self.spot_cmd_vel_sub = rospy.Subscriber(self._spot_cmd_vel, Twist, self.spot_cmd_vel_sub_callback)
#                 self.spot2haetae_cmd_vel_pub = rospy.Publisher(self._spot2haetae_cmd_vel, Twist, queue_size=10)

#                 # self.k_constant = 0.9
#                 self.k_constant = 1.5


#         return True

#     def get_obj_pose_from_wm(self, target_name):
#         blackboard = py_trees.Blackboard()
#         wm_msg = json.loads(blackboard.wm_msg.data)["world"]

#         target_pose = None

#         for wm_obj in wm_msg:
#             wm_obj_name = wm_obj["name"]
#             if wm_obj_name == target_name:
#                 target_pose = wm_obj["pose"]
#         return target_pose


#     def spot_cmd_vel_sub_callback(self, data):

#         if self.spot2haetae_cmd_vel_pub is None:
#             raise NotImplementedError()

#         x_org, y_org, z_org  = data.linear.x, data.linear.y, data.linear.z
#         theata_org = data.angular.z

#         blackboard = py_trees.Blackboard()
#         wm_msg = json.loads(blackboard.wm_msg.data)["world"]
#         spot_coord = None
#         haetae_coord = None
#         for wm_obj in wm_msg:
#             wm_obj_name = wm_obj["name"]
#             if wm_obj_name == "spot":
#                 spot_coord = wm_obj["pose"]
#             if wm_obj_name == "haetae":
#                 haetae_coord = wm_obj["pose"]
        
#         x_vel = (spot_coord[0] - self.spot_haetae_x_diff - haetae_coord[0]) * self.k_constant * -1
#         y_vel = (spot_coord[1] - self.spot_haetae_y_diff - haetae_coord[1]) * self.k_constant
        
#         send_data = Twist()
#         # send_data.linear.x = y_org
#         # send_data.linear.y = x_org
#         send_data.linear.x = y_vel
#         send_data.linear.y = x_vel
#         send_data.linear.z = z_org
#         send_data.angular.x = 0.0
#         send_data.angular.y = 0.0
#         send_data.angular.z = theata_org
#         # send_data.angular.z = 0.0

#         self.spot2haetae_cmd_vel_pub.publish(send_data)

#     def initialise(self):
#         self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
#         self.sent_goal = False
#         blackboard = py_trees.Blackboard()
#         self.drive_status_update_req(blackboard.robot_name)
#         self.arrival_status_delete_req(blackboard.robot_name)

#     def update(self):

#         blackboard = py_trees.Blackboard()
#         # listener = tf.TransformListener()
#         # self.drive_status_update_req(blackboard.robot_name)
#         # if blackboard.robot_name == 'spot':
#         #     while not rospy.is_shutdown():
#         #         try:
#         #             (pos, quat) = listener.lookupTransform("spot/base_link_plate", "box_s_grip_1", rospy.Time(0))
#         #             (pos2, quat2) = listener.lookupTransform("spot/base_link", "picking_station2_nav_1", rospy.Time(0))
                    
#         #             print("!!!!!!!!!!\n\n", pos, len(pos))
#         #             print("!!",blackboard.robot_name)
#         #             # assert len(pos) == 3
#         #             box_plate_dist = pos[0] ** 2 + pos[1] ** 2 + pos[2] ** 2
#         #             spot_to_ps2 = pos2[0] ** 2 + pos2[1] ** 2 + pos2[2] ** 2
#         #             print(box_plate_dist, spot_to_ps2)
#         #             if box_plate_dist > 1.1: ## from origin -> picking_station2
#         #                 break
#         #             elif box_plate_dist < 0.5 and spot_to_ps2 < 0.7: ## stay at picking_station2
#         #                 break
#         #         except:
#         #             print("111111!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!23232\n")
#         #             pass

#         # elif blackboard.robot_name == 'haetae':
#         #     pass
        
#         # elif blackboard.robot_name == 'stretch':
#         #     pass

#         # else:
#         #     raise NotImplementedError()


#         self.logger.debug("%s.update()" % self.__class__.__name__)

#         if self.cmd_req is None:
#             self.feedback_message = \
#               "No action client, did you call setup() on your tree?"
#             return py_trees.Status.FAILURE

#         if not self.sent_goal:
#             if type(self.action_goal['pose']) is geometry_msgs.msg._Pose.Pose:
#                 goal = {'x': self.action_goal['pose'].position.x,
#                         'y': self.action_goal['pose'].position.y,
#                         'z': self.action_goal['pose'].position.z,
#                         'qx': self.action_goal['pose'].orientation.x,
#                         'qy': self.action_goal['pose'].orientation.y,
#                         'qz': self.action_goal['pose'].orientation.z,
#                         'qw': self.action_goal['pose'].orientation.w,}
#             else:
#                 blackboard = py_trees.Blackboard()
#                 ps = blackboard.get(self.action_goal['pose'])
#                 goal = {'x': ps.position.x,
#                         'y': ps.position.y,
#                         'z': ps.position.z,
#                         'qx': ps.orientation.x,
#                         'qy': ps.orientation.y,
#                         'qz': ps.orientation.z,
#                         'qw': ps.orientation.w,}

#             cmd_str = json.dumps({'action_type': 'moveBase',
#                                   'frame': self._world_frame_id,
#                                   'goal': json.dumps(goal),
#                                   'timeout': 10.,
#                                   'no_wait': True})

#             ret = self.cmd_req(cmd_str)
#             print("(MOVEBASE update) goal: ", goal)
#             if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
#                 self.feedback_message = "failed to execute"
#                 self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
#                 # self.drive_status_update_req(blackboard.robot_name)
#                 return py_trees.common.Status.FAILURE
            
#             self.sent_goal        = True
#             self.feedback_message = "Sending a navigation goal"
#             return py_trees.common.Status.RUNNING

#         msg = self.status_req()
#         d = json.loads(msg.data)
#         state = d['state']
        
#         if state in [GoalStatus.ABORTED, GoalStatus.PREEMPTED, GoalStatus.REJECTED]:
#             self.feedback_message = "FAILURE"
#             self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.FAILURE, self.feedback_message))
#             # self.drive_status_update_req(blackboard.robot_name)
#             return py_trees.common.Status.FAILURE

#         if state == GoalStatus.SUCCEEDED:
#             self.feedback_message = "SUCCESSFUL"
#             self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.SUCCESS, self.feedback_message))
            
#             req_data = String_Dup_NoneRequest()
#             req_data.data1 = self.destination
#             req_data.data2 = blackboard.robot_name
        
#             self.arrival_status_update_req(req_data)

#             if self.is_collab == True:
#                 req_data2 = String_Dup_NoneRequest()
#                 req_data2.data1 = self.destination
#                 req_data2.data2 = "haetae"
#                 self.arrival_status_update_req(req_data2)

#             return py_trees.common.Status.SUCCESS
#         else:
#             return py_trees.common.Status.RUNNING

        
#     def terminate(self, new_status):
#         msg = self.status_req()
#         d = json.loads(msg.data)
#         if d['state'] == GoalStatus.ACTIVE:
#             self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        
#         blackboard = py_trees.Blackboard()
#         self.drive_status_update_req(blackboard.robot_name)

#         if self.is_collab == True:
#             if blackboard.robot_name == "spot":
#                 send_data = Twist()
#                 send_data.linear.x = 0.0
#                 send_data.linear.y = 0.0
#                 send_data.linear.z = 0.0
#                 send_data.angular.x = 0.0
#                 send_data.angular.y = 0.0
#                 send_data.angular.z = 0.0
#                 self.spot2haetae_cmd_vel_pub.publish(send_data)

#         return

class MOVEB(py_trees.behaviour.Behaviour):
    """
    Move Base
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, idx='', action_goal=None, destination=None):
        super(MOVEB, self).__init__(name=name)

        # self.topic_name = topic_name
        # self.controller_ns = controller_ns
        self.action_goal = action_goal
        self.sent_goal   = False
        self._world_frame_id = rospy.get_param("/world_frame", 'map')
        self.cmd_req     = None
        self.idx = idx
        self._drive_status_update_srv_channel = "/update_robot_drive_state"
        rospy.loginfo(f'{self.__class__.__name__}.__init__() called')

        self._spot_cmd_vel = "/spot/cmd_vel"

        self.destination = destination
        self._arrival_state_udpate_srv_channel = "/update_arrival_state"
        self._arrival_state_delete_srv_channel = '/delete_arrival_state'

    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("move_base_client/command")
        self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
        rospy.wait_for_service("move_base_client/status")
        self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)

        rospy.wait_for_service(self._drive_status_update_srv_channel)
        self.drive_status_update_req = rospy.ServiceProxy(self._drive_status_update_srv_channel, String_None)

        rospy.wait_for_service(self._arrival_state_udpate_srv_channel)
        self.arrival_status_update_req = rospy.ServiceProxy(self._arrival_state_udpate_srv_channel, String_Dup_None)

        rospy.wait_for_service(self._arrival_state_delete_srv_channel)
        self.arrival_status_delete_req = rospy.ServiceProxy(self._arrival_state_delete_srv_channel, String_None)

        blackboard = py_trees.Blackboard()
        rospy.loginfo(f'{self.__class__.__name__}.setup() called')
        return True

    def get_obj_pose_from_wm(self, target_name):
        blackboard = py_trees.Blackboard()
        wm_msg = json.loads(blackboard.wm_msg.data)["world"]

        target_pose = None

        for wm_obj in wm_msg:
            wm_obj_name = wm_obj["name"]
            if wm_obj_name == target_name:
                target_pose = wm_obj["pose"]
        return target_pose

    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        rospy.loginfo(f"{self.__class__.__name__}.intialise() called")
        self.sent_goal = False
        blackboard = py_trees.Blackboard()
        self.drive_status_update_req(blackboard.robot_name)
        self.arrival_status_delete_req(blackboard.robot_name)

    def update(self):
        blackboard = py_trees.Blackboard()
        ticket = blackboard.get('Plan'+self.idx+'/ticket')
        print(f"(MOVEBASE update) ticket: {ticket}")
        docking = (ticket == 0)
        
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "No action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE
        rospy.loginfo(f'{self.__class__.__name__}.update() called')

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
                # self.drive_status_update_req(blackboard.robot_name)
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
            # self.drive_status_update_req(blackboard.robot_name)
            return py_trees.common.Status.FAILURE

        if state == GoalStatus.SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.SUCCESS, self.feedback_message))
            
            req_data = String_Dup_NoneRequest()
            req_data.data1 = self.destination
            req_data.data2 = blackboard.robot_name
        
            self.arrival_status_update_req(req_data)


            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        
        blackboard = py_trees.Blackboard()
        self.drive_status_update_req(blackboard.robot_name)
        return

class MOVEBCOLLAB(py_trees.behaviour.Behaviour):
    """
    Move Base
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None, is_collab=False, destination=None):
        super(MOVEBCOLLAB, self).__init__(name=name)
        # self.topic_name = topic_name
        # self.controller_ns = controller_ns
        self.action_goal = action_goal
        self.sent_goal   = False
        self._world_frame_id = rospy.get_param("/world_frame", 'map')
        self.cmd_req     = None

        self._drive_status_update_srv_channel = "/update_robot_drive_state"
        self._spot_cmd_vel = "/spot/cmd_vel"
        self._spot2haetae_cmd_vel = "sibal/cmd_vel"

        self.destination = destination
        self._arrival_state_udpate_srv_channel = "/update_arrival_state"
        self._arrival_state_delete_srv_channel = '/delete_arrival_state'

        self._lock = threading.Lock()

    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("move_base_client/command")
        self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
        rospy.wait_for_service("move_base_client/status")
        self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)

        rospy.wait_for_service(self._drive_status_update_srv_channel)
        self.drive_status_update_req = rospy.ServiceProxy(self._drive_status_update_srv_channel, String_None)

        rospy.wait_for_service(self._arrival_state_udpate_srv_channel)
        self.arrival_status_update_req = rospy.ServiceProxy(self._arrival_state_udpate_srv_channel, String_Dup_None)
        # self.arrival_status_update_req = rospy.ServiceProxy(self._arrival_state_udpate_srv_channel, String_None)

        rospy.wait_for_service(self._arrival_state_delete_srv_channel)
        self.arrival_status_delete_req = rospy.ServiceProxy(self._arrival_state_delete_srv_channel, String_None)

        # spot_init_pose = self.get_obj_pose_from_wm("spot")
        # haeate_init_pose = self.get_obj_pose_from_wm("haetae")
        # self.spot_haetae_x_diff = spot_init_pose[0] - haeate_init_pose[0]
        # self.spot_haetae_y_diff = spot_init_pose[1] - haeate_init_pose[1]

        # blackboard = py_trees.Blackboard()
        # self.spot_cmd_vel_sub = None
        # self.spot2haetae_cmd_vel_pub = None
        # if self.is_collab == True:
        #     if blackboard.robot_name == "spot":
        self.spot_cmd_vel_sub = rospy.Subscriber(self._spot_cmd_vel, Twist, self.spot_cmd_vel_sub_callback)
        self.spot2haetae_cmd_vel_pub = rospy.Publisher(self._spot2haetae_cmd_vel, Twist, queue_size=10)

        # self.k_constant = 0.9
        self.k_constant = 1.5

        self.spot_theta_cmd_vel = 0.0

        return True

    def get_obj_pose_from_wm(self, target_name):
        blackboard = py_trees.Blackboard()
        wm_msg = json.loads(blackboard.wm_msg.data)["world"]

        target_pose = None

        for wm_obj in wm_msg:
            wm_obj_name = wm_obj["name"]
            if wm_obj_name == target_name:
                target_pose = wm_obj["pose"]
        return target_pose

    def spot_cmd_vel_sub_callback(self, data):
        # x_org, y_org, z_org  = data.linear.x, data.linear.y, data.linear.z
        theata_org = data.angular.z
        with self._lock:
            self.spot_theta_cmd_vel = theata_org

    def publish_collab_cmd_vel(self):
        blackboard = py_trees.Blackboard()
        wm_msg = json.loads(blackboard.wm_msg.data)["world"]
        spot_coord = None
        haetae_coord = None
        for wm_obj in wm_msg:
            wm_obj_name = wm_obj["name"]
            if wm_obj_name == "spot":
                spot_coord = wm_obj["pose"]
            if wm_obj_name == "haetae":
                haetae_coord = wm_obj["pose"]
        
        x_vel = (spot_coord[0] - self.spot_haetae_x_diff - haetae_coord[0]) * self.k_constant * -1
        y_vel = (spot_coord[1] - self.spot_haetae_y_diff - haetae_coord[1]) * self.k_constant
        
        send_data = Twist()
        send_data.linear.x = y_vel
        send_data.linear.y = x_vel
        send_data.linear.z = 0.0
        send_data.angular.x = 0.0
        send_data.angular.y = 0.0
        with self._lock:
            send_data.angular.z = self.spot_theta_cmd_vel
 
        self.spot2haetae_cmd_vel_pub.publish(send_data)

    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        blackboard = py_trees.Blackboard()
        self.drive_status_update_req(blackboard.robot_name)
        self.arrival_status_delete_req(blackboard.robot_name)

        spot_init_pose = self.get_obj_pose_from_wm("spot")
        haeate_init_pose = self.get_obj_pose_from_wm("haetae")
        self.spot_haetae_x_diff = spot_init_pose[0] - haeate_init_pose[0]
        self.spot_haetae_y_diff = spot_init_pose[1] - haeate_init_pose[1]

    def update(self):

        blackboard = py_trees.Blackboard()
        
        ### only this? ## DM
#         ticket = blackboard.get('Plan'+self.idx+'/ticket')
#         print(f"(MOVEBASE update) ticket: {ticket}")
#         docking = (ticket == 0)
      
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
                                  'no_wait': True,
                                  'docking': docking})

            ret = self.cmd_req(cmd_str)
            print("(MOVEBASE update) goal: ", goal)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                # self.drive_status_update_req(blackboard.robot_name)
                return py_trees.common.Status.FAILURE
            
            self.sent_goal        = True
            self.feedback_message = "Sending a navigation goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        
        print(f"[MoveBase subtree] goal state: {state}")
        if state in [GoalStatus.ABORTED, GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.RECALLED]:
            print(f"[MoveBase subtree] goal state: FAILED")
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.FAILURE, self.feedback_message))
            # self.drive_status_update_req(blackboard.robot_name)
            return py_trees.common.Status.FAILURE

        self.publish_collab_cmd_vel()

        if state == GoalStatus.SUCCEEDED:
            print(f"[MoveBase subtree] goal state: SUCCEEDED")
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.SUCCESS, self.feedback_message))
            
            req_data = String_Dup_NoneRequest()
            req_data.data1 = self.destination
            req_data.data2 = blackboard.robot_name
        
            self.arrival_status_update_req(req_data)


            req_data2 = String_Dup_NoneRequest()
            req_data2.data1 = self.destination
            req_data2.data2 = "haetae"
            self.arrival_status_update_req(req_data2)

            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        rospy.loginfo(f'{self.__class__.__name__}.terminate() called')
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        
        blackboard = py_trees.Blackboard()
        self.drive_status_update_req(blackboard.robot_name)

        if blackboard.robot_name == "spot":
            send_data = Twist()
            send_data.linear.x = 0.0
            send_data.linear.y = 0.0
            send_data.linear.z = 0.0
            send_data.angular.x = 0.0
            send_data.angular.y = 0.0
            send_data.angular.z = 0.0
            self.spot2haetae_cmd_vel_pub.publish(send_data)

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

