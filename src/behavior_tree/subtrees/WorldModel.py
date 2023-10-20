import copy, sys
import numpy as np
import json

import rospy
import py_trees
import PyKDL
import tf

from geometry_msgs.msg import Pose
from complex_action_client import misc
from riro_srvs.srv import String_None, String_String, String_Pose, String_PoseResponse


class REMOVE(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None,
                 topic_name="", controller_ns=""):
        super(REMOVE, self).__init__(name=name)

        self.topic_name    = topic_name
        self.action_goal   = action_goal
        self.sent_goal     = False
        self.cmd_req       = None


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("/remove_wm_object")
        self.cmd_req = rospy.ServiceProxy("/remove_wm_object", String_None)
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
            # cmd_str = json.dumps({'': 'gripperGotoPos',
            #                       'goal': self.action_goal})
            self.cmd_req(self.action_goal['obj_name'])
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
        self._base_pose_srv_channel = '/get_object_base_pose'
        self._height_srv_channel = '/get_object_height'
        self._rnd_pose_srv_channel = '/get_object_rnd_pose'
        self._close_pose_srv_channel = '/get_object_close_pose'
        self._aside_pose_srv_channel = '/get_object_aside_pose'
        
        self._world_frame    = rospy.get_param("/world_frame", None)
        if self._world_frame is None:
            self._world_frame    = rospy.get_param("world_frame", '/base_footprint')
        self._arm_base_frame = rospy.get_param("arm_base_frame", '/ur_arm_base_link')

        self.grasp_offset_z = rospy.get_param("grasp_offset_z", 0.02)
        self.top_offset_z   = rospy.get_param("top_offset_z", 0.10)



    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        ## rospy.wait_for_service("remove_wm_object")
        ## self.cmd_req = rospy.ServiceProxy("remove_wm_object", String_None)

        rospy.wait_for_service(self._pose_srv_channel)
        self.pose_srv_req = rospy.ServiceProxy(self._pose_srv_channel, String_Pose)

        rospy.wait_for_service(self._grasp_pose_srv_channel)
        self.grasp_pose_srv_req = rospy.ServiceProxy(self._grasp_pose_srv_channel, String_Pose)

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
        self._parking_pose_srv_channel = '/get_object_parking_pose'
        
        self._world_frame   = rospy.get_param("/world_frame", None)
        self.torso_offset_x  = rospy.get_param("torso_offset_x", -0.40)
        self.approaching_offset_x   = rospy.get_param("approaching_offset_x", -0.50)
        
    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        ## rospy.wait_for_service("remove_wm_object")
        ## self.cmd_req = rospy.ServiceProxy("remove_wm_object", String_None)

        rospy.wait_for_service(self._pose_srv_channel)
        self.pose_srv_req = rospy.ServiceProxy(self._pose_srv_channel, String_Pose)

        rospy.wait_for_service(self._parking_pose_srv_channel)
        self.parking_pose_srv_req = rospy.ServiceProxy(self._parking_pose_srv_channel, String_Pose)

        # get odom 2 base
        self.listener = tf.TransformListener()
        
        self.feedback_message = "{}: finished setting up".format(self.name)
        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False

        self.blackboard = py_trees.Blackboard()
        self.blackboard.set(self.name +'/near_parking_pose', Pose())
        # self.blackboard.set(self.name +'/aligned_parking_pose', Pose())
        self.blackboard.set(self.name +'/parking_pose', Pose())
        

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if not self.sent_goal:

            # Request the top surface pose of an object to WM
            obj = self.object_dict['destination']
            try:
                parking_pose = self.parking_pose_srv_req(obj).pose
            except rospy.ServiceException as e:
                self.feedback_message = "Parking Pose Srv Error"
                print("Pose Service is not available: %s"%e)
                return py_trees.common.Status.FAILURE

            offset          = PyKDL.Frame(PyKDL.Rotation.Identity(),
                                          PyKDL.Vector(self.torso_offset_x, 0, 0))
            parking_pose  = misc.pose2KDLframe(parking_pose) * offset
            offset          = PyKDL.Frame(PyKDL.Rotation.Identity(),
                                          PyKDL.Vector(self.approaching_offset_x, 0, 0))
            near_parking_pose  = parking_pose * offset

            parking_pose = misc.KDLframe2Pose(parking_pose)
            near_parking_pose = misc.KDLframe2Pose(near_parking_pose)
            
        
            # try:
            #     obj_base_pose = self.base_pose_srv_req(obj).pose
            # except rospy.ServiceException as e:
            #     self.feedback_message = "Pose Srv Error"
            #     print("Base Pose Service is not available: %s"%e)
            #     return py_trees.common.Status.FAILURE
            
            self.blackboard.set(self.name +'/parking_pose', parking_pose)
            self.blackboard.set(self.name +'/near_parking_pose', near_parking_pose)
            # rospy.set_param('obj_grasp_pose', json.dumps(misc.pose2list(obj_pose)))
            
            
            self.sent_goal        = True
            self.feedback_message = "WorldModel: successful parking pose estimation "
            return py_trees.common.Status.SUCCESS
            ## return py_trees.common.Status.RUNNING

        
        self.feedback_message = "WorldModel: successful parking pose estimation "
        return py_trees.common.Status.SUCCESS
            
    
    def terminate(self, new_status):
        return


