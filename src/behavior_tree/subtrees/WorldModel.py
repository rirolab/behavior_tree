import copy, sys
import numpy as np
import json

import rospy
import py_trees
import PyKDL
import tf

from geometry_msgs.msg import Pose, Point, Quaternion
from complex_action_client import misc
from riro_srvs.srv import String_None, String_NoneRequest, String_Pose, String_PoseResponse, Delivery_Ticket, Delivery_TicketRequest


class REMOVE(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, target=''):
        super(REMOVE, self).__init__(name=name)
        self.target        = target
        self.sent_goal     = False
        self.srv_req       = None


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("/delete_box")
        self.srv_req = rospy.ServiceProxy("/delete_box", String_None)
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.srv_req is None:
            self.feedback_message = \
              "Service not connected, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            req = String_NoneRequest()
            req.data = self.target
            self.srv_req(req)
            self.sent_goal = True
            self.feedback_message = "Sending a world_model command"
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS
            
    
    def terminate(self, new_status):
        return


class POSE_ESTIMATOR(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, object_dict=None, place_aside=False, en_random=False, en_close_pose=False):
        super(POSE_ESTIMATOR, self).__init__(name=name)

        self.object_dict = object_dict
        self.sent_goal   = False
        self.place_aside = place_aside
        self.en_random   = en_random
        self.en_close_pose = en_close_pose

        self._pose_srv_channel = '/get_object_pose'
        self._grasp_pose_srv_channel = '/get_object_grasp_pose'
        self._place_pose_srv_channel = '/get_object_place_pose'
        self._base_pose_srv_channel = '/get_object_base_pose'
        self._height_srv_channel = '/get_object_height'
        self._rnd_pose_srv_channel = '/get_object_rnd_pose'
        self._close_pose_srv_channel = '/get_object_close_pose'
        self._aside_pose_srv_channel = '/get_object_aside_pose'
        
        self._world_frame    = rospy.get_param("/world_frame", None)
        if self._world_frame is None:
            self._world_frame    = rospy.get_param("world_frame", '/base_footprint')
        self._arm_base_frame = rospy.get_param("arm_base_frame", '/ur_arm_base_link')

        self.grasp_offset_z = rospy.get_param("grasp_offset_z", 0.03)
        self.top_offset_z   = rospy.get_param("top_offset_z", 0.05)



    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        ## rospy.wait_for_service("remove_wm_object")
        ## self.cmd_req = rospy.ServiceProxy("remove_wm_object", String_None)

        rospy.wait_for_service(self._pose_srv_channel)
        self.pose_srv_req = rospy.ServiceProxy(self._pose_srv_channel, String_Pose)

        rospy.wait_for_service(self._grasp_pose_srv_channel)
        self.grasp_pose_srv_req = rospy.ServiceProxy(self._grasp_pose_srv_channel, String_Pose)

        rospy.wait_for_service(self._place_pose_srv_channel)
        self.place_pose_srv_req = rospy.ServiceProxy(self._place_pose_srv_channel, String_Pose)
        
        rospy.wait_for_service(self._base_pose_srv_channel)
        self.base_pose_srv_req = rospy.ServiceProxy(self._base_pose_srv_channel, String_Pose)
        
        rospy.wait_for_service(self._aside_pose_srv_channel)
        self.aside_pose_srv_req = rospy.ServiceProxy(self._aside_pose_srv_channel, String_Pose)
        
        rospy.wait_for_service(self._height_srv_channel)
        self.height_srv_req = rospy.ServiceProxy(self._height_srv_channel, String_Pose)

        rospy.wait_for_service(self._rnd_pose_srv_channel)
        self.rnd_pose_srv_req = rospy.ServiceProxy(self._rnd_pose_srv_channel, String_Pose)

        rospy.wait_for_service(self._close_pose_srv_channel)
        self.close_pose_srv_req = rospy.ServiceProxy(self._close_pose_srv_channel, String_Pose)

        # get odom 2 base
        self.listener = tf.TransformListener()
        
        self.feedback_message = "{}: finished setting up".format(self.name)
        return True


    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()
        self.blackboard.set(self.name +'/grasp_pose', Pose())
        self.blackboard.set(self.name +'/grasp_top_pose', Pose())
        self.blackboard.set(self.name +'/place_pose', Pose())    
        self.blackboard.set(self.name +'/place_top_pose', Pose())


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if not self.sent_goal:
            # Request the top surface pose of an object to WM
            obj = self.object_dict['target']
            try:
                obj_grasp_pose = self.grasp_pose_srv_req(obj).pose # obj grasping pose w.r.t. world frame
            except rospy.ServiceException as e:
                self.feedback_message = "Pose Srv Error"
                print("Pose Service is not available: %s"%e)
                return py_trees.common.Status.FAILURE
            
            try:
                obj_base_pose = self.base_pose_srv_req(obj).pose # object base pose w.r.t. world frame (pose attached on object bottom surface)
            except rospy.ServiceException as e:
                self.feedback_message = "Pose Srv Error"
                print("Base Pose Service is not available: %s"%e)
                return py_trees.common.Status.FAILURE
            
            try:
                obj_height = self.height_srv_req(obj).pose
                obj_height = obj_height.position.z
            except rospy.ServiceException as e:
                self.feedback_message = "Pose Srv Error"
                print("Height Service is not available: %s"%e)
                return py_trees.common.Status.FAILURE

            # from IPython import embed; embed(); sys.exit()
            pos = None
            while not rospy.is_shutdown():
                try:
                    (pos,quat) = self.listener.lookupTransform(self._world_frame,
                                                               self._arm_base_frame,
                                                               rospy.Time(0))
                except (tf.LookupException, tf.ConnectivityException,
                        tf.ExtrapolationException):
                    self.feedback_message = "WorldModel: Exception from TF"
                    continue
                if pos is not None: break

            self.base2arm_baselink = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3]),
                PyKDL.Vector(pos[0], pos[1], pos[2]))

            # ------------ Compute -------------------------
            grasp_pose = \
            POSE_ESTIMATOR.get_grasp_pose(obj_grasp_pose, \
                                          self.base2arm_baselink, \
                                          self.grasp_offset_z) # object grasp pose w.r.t. arm base link
                                                               # Considering top_offset_z value, normally set this as object top surface pose.
            grasp_top_pose  = copy.deepcopy(grasp_pose)
        
            offset          = PyKDL.Frame(PyKDL.Rotation.Identity(),
                                          PyKDL.Vector(0, 0, self.top_offset_z))
            grasp_top_pose  = misc.pose2KDLframe(grasp_top_pose) * offset
            grasp_top_pose  = misc.KDLframe2Pose(grasp_top_pose)
        
            # grasp_top_pose.position.z += self.top_offset_z
            #from IPython import embed; embed(); sys.exit()
            self.blackboard.set(self.name +'/grasp_pose', grasp_pose)
            self.blackboard.set(self.name +'/grasp_top_pose', grasp_top_pose)
            rospy.set_param('grasp_pose', json.dumps(misc.pose2list(grasp_pose)))
            rospy.set_param('grasp_top_pose', json.dumps(misc.pose2list(grasp_top_pose)))
            rospy.set_param('obj_base_pose', json.dumps(misc.pose2list(obj_base_pose)))
            rospy.set_param('obj_grasp_pose', json.dumps(misc.pose2list(obj_grasp_pose)))
            
            
            # Place pose
            if 'destination' in list(self.object_dict.keys()):
                self.feedback_message = "getting the destination pose"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                destination = self.object_dict['destination']
                try:
                    # Find a location on the destination top surface
                    if isinstance(destination, list): # For Arch Construction
                        dst_pose1 = misc.pose2array(self.pose_srv_req(destination[0]).pose)
                        dst_pose2 = misc.pose2array(self.pose_srv_req(destination[1]).pose)
                        dst_quat = (dst_pose1 + dst_pose2) / 2
                        rotation_angle = np.arctan2(dst_pose1[1]-dst_pose2[1], dst_pose1[0]-dst_pose2[0]) + np.pi/2
                        dst_pose = misc.list2Pose([dst_quat[0], dst_quat[1], dst_quat[2], 0., 0., rotation_angle])
                        print("[mid pose] ", dst_pose)
                    elif self.en_random:
                        dst_pose = self.rnd_pose_srv_req(json.dumps(self.object_dict)).pose
                    elif self.en_close_pose:
                        dst_pose = self.close_pose_srv_req(json.dumps(self.object_dict)).pose
                    elif self.place_aside:
                        dst_pose = self.aside_pose_srv_req(destination).pose
                    else:
                        dst_pose = self.pose_srv_req(destination).pose
                except rospy.ServiceException as e:
                    print("Pose Service is not available: %s"%e)
                    self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                    return py_trees.common.Status.FAILURE

                if 'destination_offset' in list(self.object_dict.keys()):
                    offset = self.object_dict['destination_offset']
                    offset_frame = PyKDL.Frame(PyKDL.Rotation.RotZ(offset[3]),
                                               PyKDL.Vector(offset[0],
                                                            offset[1],
                                                            offset[2]))

                    dst_frame = misc.pose2KDLframe(dst_pose)*offset_frame
                    dst_pose = misc.KDLframe2Pose(dst_frame)
                place_pose = POSE_ESTIMATOR.get_place_pose(dst_pose, \
                                                           self.base2arm_baselink, \
                                                           obj_grasp_pose, obj_base_pose, \
                                                           self.grasp_offset_z)
                
                ## from IPython import embed; embed(); sys.exit()
                
                # the sliding motion 
                if self.en_close_pose:
                    place_pose.position.z = grasp_pose.position.z

                place_top_pose = copy.deepcopy(place_pose)
                offset          = PyKDL.Frame(PyKDL.Rotation.Identity(),
                                PyKDL.Vector(0, 0, self.top_offset_z))
                place_top_pose  = misc.pose2KDLframe(place_top_pose) * offset
                place_top_pose  = misc.KDLframe2Pose(place_top_pose)
                # place_top_pose.position.z += self.top_offset_z

                self.blackboard.set(self.name +'/place_pose', place_pose)    
                self.blackboard.set(self.name +'/place_top_pose', place_top_pose)
                rospy.set_param('place_pose', json.dumps(misc.pose2list(place_pose)))
                rospy.set_param('place_top_pose', json.dumps(misc.pose2list(place_top_pose)))
            
            self.sent_goal        = True
            self.feedback_message = "WorldModel: successful grasp pose estimation "
            return py_trees.common.Status.SUCCESS
            ## return py_trees.common.Status.RUNNING

        
        self.feedback_message = "WorldModel: successful grasp pose estimation "
        ## if not self.sent_goal:
        ##     self.sent_goal = True
        ##     self.feedback_message = "Sending a world_model command"
        ##     return py_trees.common.Status.RUNNING
        return py_trees.common.Status.SUCCESS
            
    
    def terminate(self, new_status):
        return


    @staticmethod
    def get_grasp_pose(obj_pose, base2arm_baselink, grasp_offset_z):
        """ Return the grasp pose."""
        base2obj = misc.pose2KDLframe(obj_pose)

        # TODO: this is a fail safe code. It may need to be removed.
        # if abs(base2obj.M.UnitZ()[2]) < 0.3:
        #     if abs(base2obj.M.UnitX()[2]) < 0.3:
        #         base2obj.M.DoRotX(-np.pi/2.)
        #     else:
        #         base2obj.M.DoRotY(np.pi/2.)
                
        arm_baselink2obj = base2arm_baselink.Inverse() * base2obj

        #TODO: robot specific grasping orientation
        ## obj2grasp      = PyKDL.Frame(PyKDL.Rotation.RotX(-np.pi/2.) \
        ##                              * PyKDL.Rotation.RotY(-np.pi/2.))
        #obj2grasp      = PyKDL.Frame(PyKDL.Rotation.RotX(-np.pi/2.))
        #obj2grasp      = PyKDL.Frame(PyKDL.Rotation.RotY(np.pi/2.))
        baselink2grasp = arm_baselink2obj #* obj2grasp

        # Grasping pose
        # grasp_pose      = misc.KDLframe2Pose(baselink2grasp)
        # grasp_pose.position.z -= grasp_offset_z        
        offset = PyKDL.Frame(PyKDL.Rotation.Identity(),
                             PyKDL.Vector(0, 0, -grasp_offset_z))
        grasp_pose      = baselink2grasp * offset
        grasp_pose      = misc.KDLframe2Pose(grasp_pose)
        return grasp_pose

    @staticmethod
    def get_place_pose(destination_pose, base2arm_baselink, \
                       obj_grip_pose, obj_base_pose, grasp_offset_z):
        """ return the place pose """
        base2obj         = misc.pose2KDLframe(destination_pose)
        arm_baselink2obj = base2arm_baselink.Inverse() * base2obj
        # arm_baselink2obj.M = misc.pose2KDLframe(grasp_pose).M
        
        base2obj_grip     = misc.pose2KDLframe(obj_grip_pose)
        base2obj_base     = misc.pose2KDLframe(obj_base_pose)
        obj_base2obj_grip = base2obj_base.Inverse() * base2obj_grip
        
        baselink2obj = arm_baselink2obj * obj_base2obj_grip
        # place_pose     = misc.KDLframe2Pose(arm_baselink2obj)
        offset = PyKDL.Frame(PyKDL.Rotation.Identity(),
                             PyKDL.Vector(0, 0, -grasp_offset_z))
        place_pose      = baselink2obj * offset
        place_pose      = misc.KDLframe2Pose(place_pose)
        # for the object hold by the hand
        # place_pose.position.z += obj_height
        # place_pose.position.z -= grasp_offset_z
        return place_pose 


class PARKING_POSE_ESTIMATOR(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, object_dict=None):
        super(PARKING_POSE_ESTIMATOR, self).__init__(name=name)

        self.object_dict = object_dict
        self.sent_goal   = False
        
        self._pose_srv_channel = '/get_object_pose'
        self._parking_pose_srv_channel = '/get_parking_ticket'
        
        self._world_frame   = rospy.get_param("/world_frame", None)
        self._home_pose     = eval(rospy.get_param('home_config', str([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])))
        self._home_pose     = misc.list_rpy2list_quat(self._home_pose)
        
    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        ## rospy.wait_for_service("remove_wm_object")
        ## self.cmd_req = rospy.ServiceProxy("remove_wm_object", String_None)

        rospy.wait_for_service(self._pose_srv_channel)
        self.pose_srv_req = rospy.ServiceProxy(self._pose_srv_channel, String_Pose)
        rospy.wait_for_service(self._parking_pose_srv_channel)
        self.parking_pose_srv_req = rospy.ServiceProxy(self._parking_pose_srv_channel, Delivery_Ticket)

        self.feedback_message = "{}: finished setting up".format(self.name)
        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()
        self.blackboard.set(self.name +'/near_parking_pose', Pose())
        self.blackboard.set(self.name +'/parking_pose', Pose())
        self.blackboard.set(self.name +'/home_pose', Pose(Point(self._home_pose[0],self._home_pose[1],self._home_pose[2]),
                                                          Quaternion(self._home_pose[3],self._home_pose[4],self._home_pose[5],self._home_pose[6])))
    
        

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if not self.sent_goal:

            # Request the top surface pose of an object to WM
            obj = self.object_dict['destination']
            if obj == 'home':
                self.blackboard.set(self.name +'/ticket', 1)
                self.blackboard.set(self.name +'/home_pose', Pose(Point(self._home_pose[0],self._home_pose[1],self._home_pose[2]),
                                                          Quaternion(self._home_pose[3],self._home_pose[4],self._home_pose[5],self._home_pose[6])))
                self.sent_goal        = True
                self.feedback_message = "WorldModel: successful home pose estimation "
                return py_trees.common.Status.SUCCESS
                
            try:
                req = Delivery_TicketRequest()
                req.robot = self.object_dict['robot']
                req.destination = self.object_dict['destination']
                resp = self.parking_pose_srv_req(req)
                
                ticket_order = resp.order
                parking_pose = resp.pose

            except rospy.ServiceException as e:
                self.feedback_message = "Parking Pose Srv Error"
                print("Pose Service is not available: %s"%e)
                return py_trees.common.Status.FAILURE
            
            self.blackboard.set(self.name +'/ticket', ticket_order)
            self.blackboard.set(self.name +'/parking_pose', parking_pose)
            
            self.sent_goal        = True
            self.feedback_message = "WorldModel: successful parking pose estimation "
            return py_trees.common.Status.SUCCESS
        
        self.feedback_message = "WorldModel: successful parking pose estimation "
        return py_trees.common.Status.SUCCESS
            
    
    def terminate(self, new_status):
        return

## Haetae wait for Spot to come ## 
class SYNC_POSE_ESTIMATOR(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, distance_criteria, object_dict=None, wait_spot_drive=False, smaller_than_criteria=True):
        super(SYNC_POSE_ESTIMATOR, self).__init__(name=name)

        self.object_dict = object_dict
        self.sent_goal   = False
        self.distance_criteria = distance_criteria
        self.wait_spot_drive = wait_spot_drive
        self.smaller_than_criteria = smaller_than_criteria
        self._pose_srv_channel = '/get_object_pose'
        # self._parking_pose_srv_channel = '/get_object_parking_pose'
        self._sync_pose_srv_channel = '/get_sync_pose'

        self._world_frame   = rospy.get_param("/world_frame", None)
        self._manip_status_update_srv_channel = "/update_robot_manip_state"
        
    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        ## rospy.wait_for_service("remove_wm_object")
        ## self.cmd_req = rospy.ServiceProxy("remove_wm_object", String_None)

        rospy.wait_for_service(self._pose_srv_channel)
        self.pose_srv_req = rospy.ServiceProxy(self._pose_srv_channel, String_Pose)

        # rospy.wait_for_service(self._parking_pose_srv_channel)
        # self.parking_pose_srv_req = rospy.ServiceProxy(self._parking_pose_srv_channel, String_Pose)

        rospy.wait_for_service(self._sync_pose_srv_channel)
        self.sync_srv_req = rospy.ServiceProxy(self._sync_pose_srv_channel, String_Pose)

        # get odom 2 base
        self.listener = tf.TransformListener()
        
        self.feedback_message = "{}: finished setting up".format(self.name)

        if self.wait_spot_drive == True:
            self.is_drive = None

        self.manip_status_update_req = rospy.ServiceProxy(self._manip_status_update_srv_channel, String_None)

        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()
        self.blackboard.set(self.name +'/sync_pose_target1', Pose())
        self.blackboard.set(self.name +'/sync_pose_target2', Pose())        
        
        self.manip_status_update_req(self.blackboard.robot_name)

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # if not self.sent_goal:
        # Request the top surface pose of an object to WM
        obj1 = self.object_dict['target1']
        obj2 = self.object_dict['target2']
        # try:
        #     sync_pose1 = self.sync_srv_req(obj1).pose
        #     sync_pose2 = self.sync_srv_req(obj2).pose
        # except rospy.ServiceException as e:
        #     self.feedback_message = "Syncing Pose Srv Error"
        #     print("Pose Service is not available!!!: %s"%e)
        #     return py_trees.common.Status.FAILURE

        # self.blackboard.set(self.name +'/sync_pose_target1', sync_pose1)
        # self.blackboard.set(self.name +'/sync_pose_target2', sync_pose2)

        # print("!!!!!!!!!!!!!!!!!14141431!!!!\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n", sync_pose1, sync_pose2)
        # print(sync_pose1.position.x, sync_pose2.position.y)
        
        # wm_msg = self.blackboard.wm_msg.data["world"]
        wm_msg = json.loads(self.blackboard.wm_msg.data)["world"]
    
        for wm_obj in wm_msg:
            wm_obj_name = wm_obj['name']
            if wm_obj_name == obj1:
                obj1_pose = wm_obj['pose']
            elif wm_obj_name == obj2:
                obj2_pose = wm_obj['pose']

            if self.wait_spot_drive == True:
                if wm_obj_name == 'spot':
                    # print("#@@!@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n", wm_obj)
                    self.is_drive = wm_obj['robot_is_driving']
        
        sync_distance_bb = (obj1_pose[0] - obj2_pose[0]) ** 2 + (obj1_pose[1] - obj2_pose[1]) ** 2
        sync_distance_bb = np.sqrt(sync_distance_bb)
        
        # sync_distance = (sync_pose1.position.x - sync_pose2.position.x) ** 2 + (sync_pose1.position.y - sync_pose2.position.y) ** 2
        # sync_distance = np.sqrt(sync_distance)

        self.sent_goal        = True

        if self.smaller_than_criteria == True:
            if sync_distance_bb > self.distance_criteria:
                # print("fdqfeqqfqefwqFEQFEDQWFWFEWQ\n\n\n", sync_distance_bb)
                self.feedback_message = "SYNCING!!!!"
                return py_trees.common.Status.RUNNING
        else:
            if sync_distance_bb <= self.distance_criteria:
                # print("2222222222fdqfeqqfqefwqFEQFEDQWFWFEWQ\n\n\n", sync_distance_bb)
                self.feedback_message = "SYNCING!!!!"
                return py_trees.common.Status.RUNNING


        if (self.wait_spot_drive == True) and (self.is_drive is not None):
            if self.is_drive == True:
                return py_trees.common.Status.RUNNING


        self.feedback_message = "WorldModel: successful sync pose estimation "

        return py_trees.common.Status.SUCCESS

        
        # self.feedback_message = "WorldModel: successful sync pose estimation "
        # return py_trees.common.Status.SUCCESS


    def terminate(self, new_status):
        self.manip_status_update_req(self.blackboard.robot_name)
        return

## Spot wait for Haetae to load box_s## 
class SYNC_POSE_ESTIMATOR_SPOT(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, distance_criteria, object_dict=None, wait_spot_drive=False):
        super(SYNC_POSE_ESTIMATOR_SPOT, self).__init__(name=name)

        self.object_dict = object_dict
        self.sent_goal   = False
        self.distance_criteria = distance_criteria
        self.wait_spot_drive = wait_spot_drive

        self._pose_srv_channel = '/get_object_pose'
        # self._parking_pose_srv_channel = '/get_object_parking_pose'
        self._sync_pose_srv_channel = '/get_sync_pose'

        self._world_frame   = rospy.get_param("/world_frame", None)
        
    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)

        # get odom 2 base
        self.listener = tf.TransformListener()
        
        self.feedback_message = "{}: finished setting up".format(self.name)

        self.is_manip = False

        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()
        # self.blackboard.set(self.name +'/sync_pose_target1', Pose())
        # self.blackboard.set(self.name +'/sync_pose_target2', Pose())        
        
        

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # if not self.sent_goal:
        # Request the top surface pose of an object to WM
        obj1 = self.object_dict['target1'] ## spot
        obj2 = self.object_dict['target2'] ## haetae

        wm_msg = json.loads(self.blackboard.wm_msg.data)["world"]
    
        for wm_obj in wm_msg:
            wm_obj_name = wm_obj['name']
            if wm_obj_name == obj1:
                obj1_pose = wm_obj['pose']
            elif wm_obj_name == obj2:
                obj2_pose = wm_obj['pose']
            
            if wm_obj_name == 'box_s':
                box_pose = wm_obj['pose']

            if wm_obj_name == 'haetae':
                self.is_manip = wm_obj['robot_is_manip']
        
        sync_distance_bb = (obj1_pose[0] - obj2_pose[0]) ** 2 + (obj1_pose[1] - obj2_pose[1]) ** 2
        sync_distance_bb = np.sqrt(sync_distance_bb)
        
        spot2box_s = (obj1_pose[0] - box_pose[0])**2 + (obj1_pose[1] - box_pose[1])**2 + (obj1_pose[2] - box_pose[2])**2
        spot2box_s = np.sqrt(spot2box_s)

        self.sent_goal        = True
        print("22222&&&&&&&&&&&&&$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n\n\n\n\n\n\n\n\n\n\n\n", self.is_manip,spot2box_s, sync_distance_bb)

        # if (sync_distance_bb < 1.0) and (spot2box_s > 0.8) and ((box_pose[2] - obj1_pose[2]) > 0.4) and ((obj1_pose[0] - box_pose[0]) < 0.6):

        if sync_distance_bb < 1.5:
            if self.is_manip == True:
                self.feedback_message = "SYNCING!!!!"
                return py_trees.common.Status.RUNNING



        self.feedback_message = "WorldModel: successful sync pose estimation "
        return py_trees.common.Status.SUCCESS


    def terminate(self, new_status):
        return

## SPOT wait for haetae to load box_l
class SYNC_POSE_ESTIMATOR_SPOT2(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, distance_criteria, object_dict=None, wait_spot_drive=False):
        super(SYNC_POSE_ESTIMATOR_SPOT, self).__init__(name=name)

        self.object_dict = object_dict
        self.sent_goal   = False
        self.distance_criteria = distance_criteria
        self.wait_spot_drive = wait_spot_drive

        # self._pose_srv_channel = '/get_object_pose'
        # self._parking_pose_srv_channel = '/get_object_parking_pose'
        # self._sync_pose_srv_channel = '/get_sync_pose'

        self._world_frame   = rospy.get_param("/world_frame", None)
        
    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)

        # get odom 2 base
        self.listener = tf.TransformListener()
        
        self.feedback_message = "{}: finished setting up".format(self.name)

        self.is_manip = False

        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()
        # self.blackboard.set(self.name +'/sync_pose_target1', Pose())
        # self.blackboard.set(self.name +'/sync_pose_target2', Pose())        
        

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # if not self.sent_goal:
        # Request the top surface pose of an object to WM
        obj1 = self.object_dict['target1'] ## spot
        obj2 = self.object_dict['target2'] ## haetae

        wm_msg = json.loads(self.blackboard.wm_msg.data)["world"]
    
        for wm_obj in wm_msg:
            wm_obj_name = wm_obj['name']
            if wm_obj_name == obj1:
                obj1_pose = wm_obj['pose']
            elif wm_obj_name == obj2:
                obj2_pose = wm_obj['pose']
            
            if wm_obj_name == 'box_l':
                box_pose = wm_obj['pose']

            if wm_obj_name == 'haetae':
                self.is_manip = wm_obj['robot_is_manip']
        
        sync_distance_bb = (obj1_pose[0] - obj2_pose[0]) ** 2 + (obj1_pose[1] - obj2_pose[1]) ** 2
        sync_distance_bb = np.sqrt(sync_distance_bb)
        
        spot2box_s = (obj1_pose[0] - box_pose[0])**2 + (obj1_pose[1] - box_pose[1])**2 + (obj1_pose[2] - box_pose[2])**2
        spot2box_s = np.sqrt(spot2box_s)

        self.sent_goal        = True
        print("22222&&&&&&&&&&&&&$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n\n\n\n\n\n\n\n\n\n\n\n", self.is_manip,spot2box_s, sync_distance_bb)

        # if (sync_distance_bb < 1.0) and (spot2box_s > 0.8) and ((box_pose[2] - obj1_pose[2]) > 0.4) and ((obj1_pose[0] - box_pose[0]) < 0.6):

        if sync_distance_bb < 1.5:
            if self.is_manip == True:
                self.feedback_message = "SYNCING!!!!"
                return py_trees.common.Status.RUNNING



        self.feedback_message = "WorldModel: successful sync pose estimation "
        return py_trees.common.Status.SUCCESS


    def terminate(self, new_status):
        return


## Haetae wait til Spot move ## 
class SYNC_POSE_ESTIMATOR_HAETAE(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, distance_criteria, object_dict=None, wait_spot_drive=False):
        super(SYNC_POSE_ESTIMATOR_HAETAE, self).__init__(name=name)

        self.object_dict = object_dict
        self.sent_goal   = False
        self.distance_criteria = distance_criteria
        self.wait_spot_drive = wait_spot_drive

        self._pose_srv_channel = '/get_object_pose'
        # self._parking_pose_srv_channel = '/get_object_parking_pose'
        self._sync_pose_srv_channel = '/get_sync_pose'

        self._world_frame   = rospy.get_param("/world_frame", None)
        
    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)

        # get odom 2 base
        self.listener = tf.TransformListener()
        
        self.feedback_message = "{}: finished setting up".format(self.name)

        self.is_manip = False
        self.is_drive_wait = False

        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()
        # self.blackboard.set(self.name +'/sync_pose_target1', Pose())
        # self.blackboard.set(self.name +'/sync_pose_target2', Pose())        
        
        

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # if not self.sent_goal:
        # Request the top surface pose of an object to WM
        obj1 = self.object_dict['target1'] ## spot
        obj2 = self.object_dict['target2'] ## haetae

        wm_msg = json.loads(self.blackboard.wm_msg.data)["world"]
    
        for wm_obj in wm_msg:
            wm_obj_name = wm_obj['name']
            if wm_obj_name == obj1:
                obj1_pose = wm_obj['pose']
            elif wm_obj_name == obj2:
                obj2_pose = wm_obj['pose']
            
            if wm_obj_name == 'box_s':
                box_pose = wm_obj['pose']

            if wm_obj_name == 'spot':
                self.is_manip = wm_obj['robot_is_driving']
                self.is_drive_wait = wm_obj['robot_is_drive_wait']
        
        sync_distance_bb = (obj1_pose[0] - obj2_pose[0]) ** 2 + (obj1_pose[1] - obj2_pose[1]) ** 2
        sync_distance_bb = np.sqrt(sync_distance_bb)
        
        spot2box_s = (obj1_pose[0] - box_pose[0])**2 + (obj1_pose[1] - box_pose[1])**2 + (obj1_pose[2] - box_pose[2])**2
        spot2box_s = np.sqrt(spot2box_s)

        self.sent_goal        = True
        print("&&&&&&&&&&&&&$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n\n\n\n\n\n\n\n\n\n\n\n\n", self.is_manip,spot2box_s, sync_distance_bb)

        # if (sync_distance_bb < 1.0) and (spot2box_s > 0.8) and ((box_pose[2] - obj1_pose[2]) > 0.4) and ((obj1_pose[0] - box_pose[0]) < 0.6):

        # if sync_distance_bb < 1.0:
        if self.is_manip == True:
            self.feedback_message = "SYNCING!!!!"
            return py_trees.common.Status.RUNNING



        self.feedback_message = "WorldModel: successful sync pose estimation "
        return py_trees.common.Status.SUCCESS


    def terminate(self, new_status):
        return


## Haetae wait til Spot move ## 
class SYNC_POSE_ESTIMATOR_HAETAE2(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, destination):
        super(SYNC_POSE_ESTIMATOR_HAETAE2, self).__init__(name=name)

        self.sent_goal   = False

        self._pose_srv_channel = '/get_object_pose'
        # self._parking_pose_srv_channel = '/get_object_parking_pose'
        self._sync_pose_srv_channel = '/get_sync_pose'
        self._update_load_state_srv_channel = '/update_load_state'
        self._world_frame   = rospy.get_param("/world_frame", None)
        
        if "placing_shelf1" in destination:
            # print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!11111\n\n\n")
            nav_goal = "placing_shelf1"            
        elif "placing_shelf2" in destination:
            # print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!11111222\n\n\n")
            nav_goal = "placing_shelf2"
        elif "picking_station1" in destination:
            # print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!111133331\n\n\n")
            nav_goal = "picking_station1"
        elif "picking_station2" in destination:
            # print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!111144441\n\n\n")
            nav_goal = "picking_station2"
        else:
            # print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!11155511\n\n\n")
            raise NotImplementedError()
        self.destination = nav_goal


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)

        # get odom 2 base
        self.listener = tf.TransformListener()
        
        self.feedback_message = "{}: finished setting up".format(self.name)

        self.is_manip = False
        self.is_drive_wait = False
        self.spot_arrival_state = False

        rospy.wait_for_service(self._update_load_state_srv_channel)
        self.update_load_req = rospy.ServiceProxy(self._update_load_state_srv_channel, String_None)

        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()     
        
        self.update_load_req("spot")
        

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # if not self.sent_goal:
        # Request the top surface pose of an object to WM
        # obj1 = self.object_dict['target1'] ## spot
        # obj2 = self.object_dict['target2'] ## haetae

        wm_msg = json.loads(self.blackboard.wm_msg.data)["world"]
    
        for wm_obj in wm_msg:
            wm_obj_name = wm_obj['name']

            if wm_obj_name == self.destination:


                if "spot" in wm_obj["arrival_obj"]:
                    self.spot_arrival_state = True
        
        self.sent_goal        = True
        if self.spot_arrival_state == False:
            self.feedback_message = "SYNCING!!!!"
            return py_trees.common.Status.RUNNING

        self.feedback_message = "WorldModel: successful sync pose estimation "
        return py_trees.common.Status.SUCCESS


    def terminate(self, new_status):
        return


## wait until "target_obj" arives "placement" ## 
class SYNC_POSE_ESTIMATOR_WAIT(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, placement, target_obj):
        super(SYNC_POSE_ESTIMATOR_WAIT, self).__init__(name=name)
        # self.object_dict = object_dict
        self.sent_goal   = False


        if "placing_shelf1" in placement:
            print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!11111\n\n\n")
            nav_goal = "placing_shelf1"            
        elif "placing_shelf2" in placement:
            print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!11111222\n\n\n")
            nav_goal = "placing_shelf2"
        elif "picking_station1" in placement:
            print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!111133331\n\n\n")
            nav_goal = "picking_station1"
        elif "picking_station2" in placement:
            print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!111144441\n\n\n")
            nav_goal = "picking_station2"
        else:
            print("@@@@@@@@@@@@@@@@@@@@@@@@!!!!!11155511\n\n\n")
            raise NotImplementedError()

        self.placement = nav_goal
        self.target_obj = target_obj
        

    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        self.feedback_message = "{}: finished setting up".format(self.name)
        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()
        # self.blackboard.set(self.name +'/sync_pose_target1', Pose())
        # self.blackboard.set(self.name +'/sync_pose_target2', Pose())        
        
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        # if not self.sent_goal:
        # Request the top surface pose of an object to WM
        wm_msg = json.loads(self.blackboard.wm_msg.data)["world"]
    
        for wm_obj in wm_msg:
            wm_obj_name = wm_obj['name']
            if wm_obj_name == self.placement:
                if self.target_obj in wm_obj['arrival_obj']:
                    self.feedback_message = "WorldModel: successful sync pose estimation "
                    return py_trees.common.Status.SUCCESS
                else:
                    self.feedback_message = "SYNCING!!!!"
                    return py_trees.common.Status.RUNNING

        # # self.sent_goal        = True

        # if self.spot_arrival_state == False:
        #     self.feedback_message = "SYNCING!!!!"
        #     return py_trees.common.Status.RUNNING

        # self.feedback_message = "WorldModel: successful sync pose estimation "
        # return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        return

## about load ## 
class SYNC_POSE_ESTIMATOR_LOAD(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, target_obj='spot'):
        super(SYNC_POSE_ESTIMATOR_LOAD, self).__init__(name=name)
        self.sent_goal   = False
        self.target_obj = target_obj
        

    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        self.feedback_message = "{}: finished setting up".format(self.name)
        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()    
        
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        # if not self.sent_goal:
        # Request the top surface pose of an object to WM
        wm_msg = json.loads(self.blackboard.wm_msg.data)["world"]
    
        for wm_obj in wm_msg:
            wm_obj_name = wm_obj['name']
            if wm_obj_name == self.target_obj:
                if wm_obj['is_loaded'] == True:
                    self.feedback_message = "WorldModel: successful sync pose estimation "
                    return py_trees.common.Status.SUCCESS
                else:
                    self.feedback_message = "SYNCING!!!!"
                    return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        return
