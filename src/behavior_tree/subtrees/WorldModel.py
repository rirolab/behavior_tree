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

    def __init__(self, name, object_dict=None, en_random=False, en_close_pose=False):
        super(POSE_ESTIMATOR, self).__init__(name=name)

        self.object_dict = object_dict
        self.sent_goal   = False
        self.en_random   = en_random
        self.en_close_pose = en_close_pose

        self._pose_srv_channel = '/get_object_pose'
        self._grasp_pose_srv_channel = '/get_object_grasp_pose'
        self._height_srv_channel = '/get_object_height'
        self._rnd_pose_srv_channel = '/get_object_rnd_pose'
        self._close_pose_srv_channel = '/get_object_close_pose'
        self._world_frame    = rospy.get_param("/world_frame", None)
        if self._world_frame is None:
            self._world_frame    = rospy.get_param("world_frame", '/base_footprint')
        self._arm_base_frame = rospy.get_param("arm_base_frame", '/ur_arm_base_link')

        self.grasp_offset_z = rospy.get_param("grasp_offset_z", 0.02)
        self.top_offset_z   = rospy.get_param("top_offset_z", 0.15)



    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        ## rospy.wait_for_service("remove_wm_object")
        ## self.cmd_req = rospy.ServiceProxy("remove_wm_object", String_None)

        rospy.wait_for_service(self._pose_srv_channel)
        self.pose_srv_req = rospy.ServiceProxy(self._pose_srv_channel, String_Pose)

        rospy.wait_for_service(self._grasp_pose_srv_channel)
        self.grasp_pose_srv_req = rospy.ServiceProxy(self._grasp_pose_srv_channel, String_Pose)

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
                obj_pose = self.grasp_pose_srv_req(obj).pose
            except rospy.ServiceException as e:
                self.feedback_message = "Pose Srv Error"
                print("Pose Service is not available: %s"%e)
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
            grasp_pose =\
            POSE_ESTIMATOR.get_grasp_pose(obj_pose, \
                                          self.base2arm_baselink, \
                                          self.grasp_offset_z)
            grasp_top_pose = copy.deepcopy(grasp_pose)
            grasp_top_pose.position.z += self.top_offset_z
            #from IPython import embed; embed(); sys.exit()

            self.blackboard.set(self.name +'/grasp_pose', grasp_pose)
            self.blackboard.set(self.name +'/grasp_top_pose', grasp_top_pose)

            # Place pose
            if 'destination' in list(self.object_dict.keys()):
                self.feedback_message = "getting the destination pose"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                destination = self.object_dict['destination']
                try:
                    # Find a location on the destination top surface
                    if self.en_random:
                        dst_pose = self.rnd_pose_srv_req(json.dumps(self.object_dict)).pose
                    elif self.en_close_pose:
                        dst_pose = self.close_pose_srv_req(json.dumps(self.object_dict)).pose
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
                                                           grasp_pose, \
                                                           obj_height, \
                                                           self.grasp_offset_z)

                ## from IPython import embed; embed(); sys.exit()
                
                # the sliding motion 
                if self.en_close_pose:
                    place_pose.position.z = grasp_pose.position.z

                place_top_pose = copy.deepcopy(place_pose)
                place_top_pose.position.z += self.top_offset_z

                self.blackboard.set(self.name +'/place_pose', place_pose)    
                self.blackboard.set(self.name +'/place_top_pose', place_top_pose)
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
        grasp_pose     = misc.KDLframe2Pose(baselink2grasp)
        grasp_pose.position.z -= grasp_offset_z        
        return grasp_pose

    @staticmethod
    def get_place_pose(obj_pose, base2arm_baselink, grasp_pose, \
                       obj_height, grasp_offset_z):
        """ return the place pose """
        base2obj         = misc.pose2KDLframe(obj_pose)
        arm_baselink2obj = base2arm_baselink.Inverse() * base2obj
        arm_baselink2obj.M = misc.pose2KDLframe(grasp_pose).M

        place_pose     = misc.KDLframe2Pose(arm_baselink2obj)

        # for the object hold by the hand
        place_pose.position.z += obj_height
        place_pose.position.z -= grasp_offset_z
        return place_pose 
