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
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import DoubleParameter
from complex_action_client import misc
from complex_action_client.srv import String_Int, None_String, String_IntRequest

from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse, String_Dup_None, String_Dup_NoneRequest

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
        rospy.loginfo('[subtree] movebase: setup() called.')
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("move_base_client/command", rospy.Duration(3))
        self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
        rospy.wait_for_service("move_base_client/status", rospy.Duration(3))
        self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)
        rospy.loginfo('[subtree] movebase: setup() done.')
        rospy.wait_for_service(self._drive_status_update_srv_channel, rospy.Duration(3))
        self.drive_status_update_req = rospy.ServiceProxy(self._drive_status_update_srv_channel, String_None)

        rospy.wait_for_service(self._arrival_state_udpate_srv_channel, rospy.Duration(3))
        self.arrival_status_update_req = rospy.ServiceProxy(self._arrival_state_udpate_srv_channel, String_Dup_None)

        rospy.wait_for_service(self._arrival_state_delete_srv_channel, rospy.Duration(3))
        self.arrival_status_delete_req = rospy.ServiceProxy(self._arrival_state_delete_srv_channel, String_None)

        blackboard = py_trees.Blackboard()
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
        rospy.loginfo('[subtree] movebase: initialise() called.')
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        rospy.loginfo(f"{self.__class__.__name__}.intialise() called")
        self.sent_goal = False
        blackboard = py_trees.Blackboard()
        self.drive_status_update_req(blackboard.robot_name)
        self.arrival_status_delete_req(blackboard.robot_name)

    def update(self):
        rospy.loginfo('[subtree] movebase: update() called.')
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
        
        if state in [GoalStatus.ABORTED, GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.RECALLED]:
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
            print("#####################################\n\n\n\n\n", req_data, req_data.data1 ,req_data.data2)
            # self.arrival_status_update_req(req_data)
            print("^^^^^^^^^^^^^^DONE^^^^^^^^^^^^^")

            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        
        blackboard = py_trees.Blackboard()
        # self.drive_status_update_req(blackboard.robot_name)
        return

class MOVEBCOLLAB(py_trees.behaviour.Behaviour):
    """
    Move Base
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, source, destination, idx='', action_goal=None, is_collab=False):
        super(MOVEBCOLLAB, self).__init__(name=name)
        # self.topic_name = topic_name
        # self.controller_ns = controller_ns
        self.action_goal = action_goal
        self.sent_goal   = False
        self._world_frame_id = rospy.get_param("/world_frame", 'map')
        self.cmd_req     = None
        self.idx = idx

        self._drive_status_update_srv_channel = "/update_robot_drive_state"
        self._spot_cmd_vel = "/spot/cmd_vel"
        self._spot2haetae_cmd_vel = "sibal/cmd_vel"

        if (destination == "placing_shelf1") or (destination == "placing_shelf2"):
            self.destination = destination
        else:
            raise NotImplementedError()
        if (source == "picking_station1") or (source == "picking_station2"):
            self.source = source
        else:
            raise NotImplementedError()
            
        self._arrival_state_udpate_srv_channel = "/update_arrival_state"
        self._arrival_state_delete_srv_channel = '/delete_arrival_state'

        # self._local_planner = rospy.get_param("local_planner", "sibals")
        self._local_planner = "TebLocalPlannerROS"
        self._prev_params = None

        self._prev_dict = {
            "max_vel_theta" : 0.15,
            "acc_lim_theta" : 0.05,
            # "penalty_epsilon" : 0.05,
            "yaw_goal_tolerance" : 0.05,
            "xy_goal_tolerance" : 0.05,
            "min_obstacle_dist" : 0.2
        }

        if source == "picking_station2":
            self.spot_haetae_x_diff = -0.00706
            self.spot_haetae_y_diff = 0.82705
        elif source == "picking_station1":
            self.spot_haetae_x_diff = 0.003126
            self.spot_haetae_y_diff = -0.8100396
        else:
            raise NotImplementedError()

        self._lock = threading.Lock()

    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("move_base_client/command", rospy.Duration(3))
        self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
        rospy.wait_for_service("move_base_client/status", rospy.Duration(3))
        self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)

        rospy.wait_for_service(self._drive_status_update_srv_channel, rospy.Duration(3))
        self.drive_status_update_req = rospy.ServiceProxy(self._drive_status_update_srv_channel, String_None)

        rospy.wait_for_service(self._arrival_state_udpate_srv_channel, rospy.Duration(3))
        self.arrival_status_update_req = rospy.ServiceProxy(self._arrival_state_udpate_srv_channel, String_Dup_None)
        # self.arrival_status_update_req = rospy.ServiceProxy(self._arrival_state_udpate_srv_channel, String_None)

        rospy.wait_for_service(self._arrival_state_delete_srv_channel, rospy.Duration(3))
        self.arrival_status_delete_req = rospy.ServiceProxy(self._arrival_state_delete_srv_channel, String_None)

        self.reconfigure_req_srv = rospy.ServiceProxy(f"move_base/{self._local_planner}/set_parameters", Reconfigure)
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
        self.k_constant = 3.0 #1.5

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
        # print("ssssssssss!!!!!!!!!!!!!\n\n\n\n\n\n\n", spot_coord, haetae_coord, self._local_planner) ## 6 dim XYZ + rpye

        x_delta = spot_coord[0] - haetae_coord[0]
        y_delta = spot_coord[1] - haetae_coord[1]

        ## original
        x_vel = (x_delta - self.spot_haetae_x_diff) * self.k_constant 
        y_vel = (y_delta - self.spot_haetae_y_diff) * self.k_constant * -1
        #################################################
        # print("!!!!!!!!!!!!!!\n\n\n", x_vel, y_vel)


        # spot_theta = np.abs(spot_coord[-1] + np.pi)
        # haetae_theta = haetae_coord[-1]

        # x_delta = spot_coord[0] - haetae_coord[0]
        # y_delta = spot_coord[1] - haetae_coord[1]
        # # delta_dist = np.sqrt(x_delta**2 + y_delta**2)
        # x_vel = x_delta * np.sin(spot_theta) + y_delta * np.cos(spot_theta) - self.spot_haetae_x_diff
        # x_vel = x_vel * self.k_constant

        # y_vel = x_delta * np.cos(spot_theta) - y_delta * np.sin(spot_theta) - self.spot_haetae_y_diff
        # y_vel = y_vel * self.k_constant * -1

        # print("!!!!!!!!!!!!!!22222\n\n\n", x_vel, y_vel, spot_theta)

        # y_vel = ((spot_coord[0] - self.spot_haetae_x_diff - haetae_coord[0]) * self.k_constant) * np.cos(spot_theta) + ((spot_coord[1] - self.spot_haetae_y_diff - haetae_coord[1]) * self.k_constant) * np.sin(spot_theta)

        if self.source == "picking_station2":
            y_vel = y_vel * -1
            x_vel = x_vel * -1

        if self.source == "picking_station2":
            if x_vel < 0:
                x_vel = 0
        elif self.source == "picking_station1":
            if x_vel > 0:
                x_vel = 0
        
        send_data = Twist()
        send_data.linear.x = y_vel
        send_data.linear.y = x_vel
        send_data.linear.z = 0.0
        send_data.angular.x = 0.0
        send_data.angular.y = 0.0 
        with self._lock:
            # send_data.angular.z = self.spot_theta_cmd_vel
            send_data.angular.z = 0.0

        self.spot2haetae_cmd_vel_pub.publish(send_data)

    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        blackboard = py_trees.Blackboard()
        self.drive_status_update_req(blackboard.robot_name)

        ## delete both "haetae" and "spot" from arrival_obj
        self.arrival_status_delete_req(blackboard.robot_name)
        self.arrival_status_delete_req("haetae")

        spot_init_pose = self.get_obj_pose_from_wm("spot")
        haeate_init_pose = self.get_obj_pose_from_wm("haetae")
        # self.spot_haetae_x_diff = spot_init_pose[0] - haeate_init_pose[0]
        # self.spot_haetae_y_diff = spot_init_pose[1] - haeate_init_pose[1]

        # print("SSSSSSSSSSSSSSSSS$EEEEEEEEE\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n", self.spot_haetae_x_diff, self.spot_haetae_y_diff)
        # raise NotImplementedError()

        if self._prev_params is None:
            self._prev_params = {"max_vel_theta": None, \
                                #  "acc_lim_theta": None, 
                                 "yaw_goal_tolerance": None, 
                                 "xy_goal_tolerance": None, "min_obstacle_dist": None }
            sssss = rospy.get_param(f'move_base/{self._local_planner}/footprint_model/vertices')
            # print("!!!!!!!!!!!!!!!!!!#@@@@@###$$$$$$$$$$$$$$$$$\n\n\n\n\n\n\n\n\n\n", sssss, type(sssss))/
            for k in self._prev_params.keys():
                self._prev_params[k] = rospy.get_param(f'move_base/{self._local_planner}/{k}')

    def update(self):

        blackboard = py_trees.Blackboard()
        
        ### only this? ## DM
        ticket = blackboard.get('Plan'+self.idx+'/ticket')
        print(f"(MOVEBASE update) ticket: {ticket}")
        docking = (ticket == 0)
      
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "No action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            reconfigure_req = ReconfigureRequest()
            for k in self._prev_params.keys():
                if k  == "yaw_goal_tolerance":
                    reconfigure_req.config.doubles.append(DoubleParameter(k, 0.1))
                elif k  == "xy_goal_tolerance":
                    reconfigure_req.config.doubles.append(DoubleParameter(k, 0.05))
                elif k  == "max_vel_theta" or k  == "acc_lim_theta":
                    reconfigure_req.config.doubles.append(DoubleParameter(k, 0.05))
                else:
                    reconfigure_req.config.doubles.append(DoubleParameter(k, 0.000))

            rospy.loginfo(f"[SUBTREE] MOVEBCOLLAB : wait for service (move_base/{self._local_planner}/set_parameters)")
            rospy.wait_for_service(f"move_base/{self._local_planner}/set_parameters", rospy.Duration(3))
            self.reconfigure_req_srv(reconfigure_req)

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
            
            # reconfigure_req = ReconfigureRequest()
            # for k in self._prev_params.keys():
            #     reconfigure_req.config.doubles.append(DoubleParameter(k, self._prev_params[k]))
            # self.reconfigure_req_srv(reconfigure_req)

            return py_trees.common.Status.FAILURE

        self.publish_collab_cmd_vel()

        if state == GoalStatus.SUCCEEDED:
            print(f"[MoveBase subtree] goal state: SUCCEEDED")
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.SUCCESS, self.feedback_message))
            
            if ticket == 0:
                req_data = String_Dup_NoneRequest()
                req_data.data1 = self.destination
                req_data.data2 = blackboard.robot_name
            
                self.arrival_status_update_req(req_data)


                req_data2 = String_Dup_NoneRequest()
                req_data2.data1 = self.destination
                req_data2.data2 = "haetae"

                # reconfigure_req = ReconfigureRequest()
                # for k in self._prev_params.keys():
                #     reconfigure_req.config.doubles.append(DoubleParameter(k, self._prev_params[k]))
                # self.reconfigure_req_srv(reconfigure_req)

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
        
        reconfigure_req = ReconfigureRequest()
        for k in self._prev_params.keys():
            # print("@@@@@@@@@@@@@@@@@2\n\n\n\n\n\n\n", k, self._prev_params[k])
            reconfigure_req.config.doubles.append(DoubleParameter(k, self._prev_dict[k]))
        self.reconfigure_req_srv(reconfigure_req)

        return

class MOVEBR(py_trees.behaviour.Behaviour):
    """
    Move Base Relative to Base_Link
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, idx, action_goal=None):
        super(MOVEBR, self).__init__(name=name)
        self.idx = idx
        self.action_goal = action_goal
        self.sent_goal   = False
        self._frame_id = rospy.get_param("footprint_frame", 'base_link')
        self.cmd_req     = None


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("move_base_client/command", rospy.Duration(3))
        self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
        rospy.wait_for_service("move_base_client/status", rospy.Duration(3))
        self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)
        
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        blackboard = py_trees.Blackboard()
        # ticket = blackboard.get('Plan'+self.idx+'/ticket')
        # print(f"(MOVEBASE update) ticket: {ticket}")
        # docking = (ticket == 0)
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
    Move Base
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, destination, idx='', action_goal=None, collab=False):
        super(TOUCHB, self).__init__(name=name)

        # self.topic_name = topic_name
        # self.controller_ns = controller_ns
        self.action_goal = action_goal
        self.sent_goal   = False
        self.collab = collab
        self._world_frame_id = rospy.get_param("/world_frame", 'map')
        self._local_planner = rospy.get_param("local_planner", "sibals")
        self._prev_params = dict()
        self.cmd_req     = None
        self.idx = idx

        self.destination = destination
        self._drive_status_update_srv_channel = "/update_robot_drive_state"
        self._arrival_state_udpate_srv_channel = "/update_arrival_state"
        self._arrival_state_delete_srv_channel = '/delete_arrival_state'

        rospy.loginfo(f'{self.__class__.__name__}.__init__() called')

    def setup(self, timeout):
        rospy.loginfo('[subtree] TouchBase: setup() called.')
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("move_base_client/command", rospy.Duration(3))
        self.cmd_req = rospy.ServiceProxy("move_base_client/command", String_Int)
        rospy.wait_for_service("move_base_client/status", rospy.Duration(3))
        self.status_req = rospy.ServiceProxy("move_base_client/status", None_String)
        # rospy.wait_for_service(f"{self._local_planner}/set_parameters", timeout=20.0)
        self.reconfigure_req_srv = rospy.ServiceProxy(f"move_base/{self._local_planner}/set_parameters", Reconfigure)
        rospy.loginfo('[subtree] TouchBase: setup() done.')
        rospy.wait_for_service(self._drive_status_update_srv_channel, rospy.Duration(3))
        self.drive_status_update_req = rospy.ServiceProxy(self._drive_status_update_srv_channel, String_None)

        rospy.wait_for_service(self._arrival_state_udpate_srv_channel, rospy.Duration(3))
        self.arrival_status_update_req = rospy.ServiceProxy(self._arrival_state_udpate_srv_channel, String_Dup_None)

        rospy.wait_for_service(self._arrival_state_delete_srv_channel, rospy.Duration(3))
        self.arrival_status_delete_req = rospy.ServiceProxy(self._arrival_state_delete_srv_channel, String_None)

        self.blackboard = py_trees.Blackboard()
        return True

    def initialise(self):
        rospy.loginfo('[subtree] TouchBase: initialise() called.')
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        rospy.loginfo(f"{self.__class__.__name__}.intialise() called")
        rospy.loginfo(f"[TOUCHB] initialize() called : local_planner = {self._local_planner}.")
        if self._local_planner == "TebLocalPlannerROS":
            self._prev_params = {'xy_goal_tolerance':None, 'yaw_goal_tolerance':None,'min_obstacle_dist':None}
        elif self._local_planner == "DWAPlannerROS":
            self._prev_params = dict()
        elif self._local_planner == "TrajectoryPlannerROS":
            self._prev_params = dict()
        else:
            raise NotImplementedError("Only TEB, Trajectory LocalPlanner is available")
        for k in self._prev_params.keys():
            self._prev_params[k] = rospy.get_param(f'move_base/{self._local_planner}/{k}')
        blackboard = py_trees.Blackboard()
        # self.drive_status_update_req(blackboard.robot_name)
        # self.arrival_status_delete_req(blackboard.robot_name)

    def update(self):
        blackboard = py_trees.Blackboard()
        rospy.loginfo('[subtree] Touchbase: update() called.')
        if self._local_planner == "TrajectoryPlannerROS" or self._local_planner == "DWAPlannerROS":
            return py_trees.Status.SUCCESS
        
        ticket = self.blackboard.get('Plan'+self.idx+'/ticket')
        docking = (ticket == 0)
        
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "No action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE
        rospy.loginfo(f'{self.__class__.__name__}.update() called')

        if not self.sent_goal:
            reconfigure_req = ReconfigureRequest()
            for k in self._prev_params.keys():
                # if self.robot_name == "spot":

                if k == "xy_goal_tolerance":
                    reconfigure_req.config.doubles.append(DoubleParameter(k, 0.05))
                elif k == "yaw_goal_tolerance":
                    reconfigure_req.config.doubles.append(DoubleParameter(k, 0.05))

                # if 'tolerance' in k:
                #     reconfigure_req.config.doubles.append(DoubleParameter(k, 0.01))
                else:
                    reconfigure_req.config.doubles.append(DoubleParameter(k, 0.01))
                # reconfigure_req.config.doubles.append(DoubleParameter(k, 0.03))

                # else: reconfigure_req.config.doubles.append(DoubleParameter(k, 0.05))
            rospy.loginfo(f"[SUBTREE] TOUCHB : wait for service (move_base/{self._local_planner}/set_parameters)")
            rospy.wait_for_service(f"move_base/{self._local_planner}/set_parameters", rospy.Duration(3))
            self.reconfigure_req_srv(reconfigure_req)

            if type(self.action_goal['pose']) is geometry_msgs.msg._Pose.Pose:
                goal = {'x': self.action_goal['pose'].position.x,
                        'y': self.action_goal['pose'].position.y,
                        'z': self.action_goal['pose'].position.z,
                        'qx': self.action_goal['pose'].orientation.x,
                        'qy': self.action_goal['pose'].orientation.y,
                        'qz': self.action_goal['pose'].orientation.z,
                        'qw': self.action_goal['pose'].orientation.w,}
            else:
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
                                  'docking':docking})

            if not self.collab:
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
        
        if state in [GoalStatus.ABORTED, GoalStatus.PREEMPTED, GoalStatus.REJECTED, GoalStatus.RECALLED]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.FAILURE, self.feedback_message))
            reconfigure_req = ReconfigureRequest()
            for k in self._prev_params.keys():
                reconfigure_req.config.doubles.append(DoubleParameter(k, self._prev_params[k]))
            self.reconfigure_req_srv(reconfigure_req)
            return py_trees.common.Status.FAILURE

        if state == GoalStatus.SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.SUCCESS, self.feedback_message))
            reconfigure_req = ReconfigureRequest()
            for k in self._prev_params.keys():
                reconfigure_req.config.doubles.append(DoubleParameter(k, self._prev_params[k]))
            if not self.collab: self.reconfigure_req_srv(reconfigure_req)
            req_data = String_Dup_NoneRequest()
            req_data.data1 = self.destination
            req_data.data2 = blackboard.robot_name
            print("#####################################\n\n\n\n\n", req_data, req_data.data1 ,req_data.data2)
            self.arrival_status_update_req(req_data)
            print("^^^^^^^^^^^^^^DONE^^^^^^^^^^^^^")

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


class TOUCHCOLLAB(py_trees.behaviour.Behaviour):
    """
    Move Base
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, idx='', action_goal=None):
        super(TOUCHCOLLAB, self).__init__(name=name)

        # self.topic_name = topic_name
        # self.controller_ns = controller_ns
        self.mode = action_goal['mode']
        self.sent_goal   = False
        self._local_planner = rospy.get_param("local_planner", "sibals")
        self._start_params = {'xy_goal_tolerance':0.01, 'yaw_goal_tolerance':0.01,'min_obstacle_dist':0.01}
        self._end_params = {'xy_goal_tolerance':0.3, 'yaw_goal_tolerance':0.2,'min_obstacle_dist':0.2}
        self.idx = idx
        rospy.loginfo(f'{self.__class__.__name__}.__init__() called')

    def setup(self, timeout):
        rospy.loginfo('[subtree] TouchCollab: setup() called.')
        self.feedback_message = "{}: setup".format(self.name)
        self.reconfigure_req_srv = rospy.ServiceProxy(f"move_base/{self._local_planner}/set_parameters", Reconfigure)
        rospy.loginfo('[subtree] TouchCollab: setup() done.')
        self.blackboard = py_trees.Blackboard()
        return True

    def initialise(self):
        rospy.loginfo('[subtree] TouchBase: initialise() called.')
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        rospy.loginfo(f"{self.__class__.__name__}.intialise() called")


    def update(self):
        rospy.loginfo('[subtree] TouchCollab: update() called.')
        if self._local_planner == "TrajectoryPlannerROS" or self._local_planner == "DWAPlannerROS":
            return py_trees.Status.SUCCESS
        self.logger.debug("%s.update()" % self.__class__.__name__)
        rospy.loginfo(f'{self.__class__.__name__}.update() called')

        if not self.sent_goal:
            reconfigure_req = ReconfigureRequest()
            params = None
            if self.mode == 'start':
                params = self._start_params
            elif self.mode == 'end':
                params = self._end_params
            for k,v in params.items():
                reconfigure_req.config.doubles.append(DoubleParameter(k, v))
            rospy.loginfo(f"[SUBTREE] TOUCHB : wait for service (move_base/{self._local_planner}/set_parameters)")
            rospy.wait_for_service(f"move_base/{self._local_planner}/set_parameters", rospy.Duration(3))
            self.reconfigure_req_srv(reconfigure_req)
            self.sent_goal        = True
            self.feedback_message = "Change move_base parameters"
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.SUCCESS

        
    def terminate(self, new_status):
        # blackboard = py_trees.Blackboard()
        # self.drive_status_update_req(blackboard.robot_name)
        return
