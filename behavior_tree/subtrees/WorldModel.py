import copy, sys
import numpy as np
import json
import typing
import time

import py_trees
import py_trees_ros
import PyKDL
import rclpy
#import py_trees.console as console
from py_trees_ros import exceptions, utilities
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from tf2_ros import TransformException



from geometry_msgs.msg import Pose
from complex_action_client import misc
from riro_srvs.srv import StringInt, StringPose


class REMOVE(py_trees.behaviour.Behaviour):
    """
    Remove an object from the world model.

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None):
        super(REMOVE, self).__init__(name=name)

        self.action_goal   = action_goal
        self.sent_goal     = False
        self.cmd_req       = None


    def setup(self,
              node: typing.Optional[rclpy.node.Node]=None,
              timeout: float=py_trees.common.Duration.INFINITE):
        self.node = node
        self.feedback_message = "{}: setup".format(self.name)
        self.cmd_req = self.node.create_client(String_None, "/remove_wm_object", qos_profile=rclpy.qos.qos_profile_services_default)
        if not self.cmd_req.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError('remove wm service not available, waiting again...')
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
            req = StringNone.Request(data=self.action_goal['obj_name'])
            future = self.cmd_req.call_async(req)

            while rclpy.ok():
                if future.done():
                    break
                rclpy.spin_once(self.node, timeout_sec=0)
                time.sleep(0.05)
            
            
            self.sent_goal = True
            self.feedback_message = "Sending a world_model command"
            return py_trees.common.Status.RUNNING

        return py_trees.common.Status.SUCCESS
            
    
    def terminate(self, new_status):
        return


class POSE_ESTIMATOR(py_trees.behaviour.Behaviour):
    """
    Obtain the target object

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, object_dict=None, en_random=False, en_close_pose=False,
                    **kwargs):
        super(POSE_ESTIMATOR, self).__init__(name=name)

        self.object_dict   = object_dict
        self.sent_goal     = False
        self.en_random     = en_random
        self.en_close_pose = en_close_pose
        self.setup_flag    = False
        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer     = kwargs['tf_buffer']


    def setup(self,
              node: typing.Optional[rclpy.node.Node]=None,
              timeout: float=py_trees.common.Duration.INFINITE):
        """ """
        self.feedback_message = "{}: setup".format(self.name)
        self.node = node

        self._pose_srv_channel       = self.node.get_parameter("pose_srv_channel").get_parameter_value().string_value
        self._grasp_pose_srv_channel = self.node.get_parameter("grasp_pose_srv_channel").get_parameter_value().string_value
        self._height_srv_channel     = self.node.get_parameter("height_srv_channel").get_parameter_value().string_value
        self._rnd_pose_srv_channel   = self.node.get_parameter("rnd_pose_srv_channel").get_parameter_value().string_value
        self._close_pose_srv_channel = self.node.get_parameter("close_pose_srv_channel").get_parameter_value().string_value
        
        self._world_frame    = self.node.get_parameter("world_frame").get_parameter_value().string_value
        self._arm_base_frame = self.node.get_parameter("arm_base_frame").get_parameter_value().string_value

        self.grasp_offset_z = self.node.get_parameter_or("grasp_offset_z", 0.02).get_parameter_value().double_value
        self.top_offset_z   = self.node.get_parameter_or("top_offset_z", 0.15).get_parameter_value().double_value

        self.pose_srv_req = self.node.create_client(StringPose, \
                                self._pose_srv_channel,
                                callback_group=self.callback_group,
                                qos_profile=rclpy.qos.qos_profile_services_default)
        if not self.pose_srv_req.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError('[{}] service not available, waiting again...'.format(self._pose_srv_channel))
        
        self.grasp_pose_srv_req = self.node.create_client(StringPose, \
                                    self._grasp_pose_srv_channel,
                                    callback_group=self.callback_group,
                                    qos_profile=rclpy.qos.qos_profile_services_default)
        if not self.grasp_pose_srv_req.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError('[{}] service not available, waiting again...'.format(self._grasp_pose_srv_channel))
        
        self.height_srv_req = self.node.create_client(StringPose, \
                                        self._height_srv_channel,
                                        callback_group=self.callback_group,
                                        qos_profile=rclpy.qos.qos_profile_services_default)
        if not self.height_srv_req.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError('[{}] service not available, waiting again...'.format(self._height_srv_channel))

        self.rnd_pose_srv_req = self.node.create_client(StringPose, \
                                            self._rnd_pose_srv_channel,
                                            callback_group=self.callback_group,
                                            qos_profile=rclpy.qos.qos_profile_services_default)
        if not self.rnd_pose_srv_req.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError('[{}] service not available, waiting again...'.format(self._rnd_pose_srv_channel))
        
        self.close_pose_srv_req = self.node.create_client(StringPose, \
                                            self._close_pose_srv_channel,
                                            callback_group=self.callback_group,
                                            qos_profile=rclpy.qos.qos_profile_services_default)
        if not self.close_pose_srv_req.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError('[{}] service not available, waiting again...'.format(self._close_pose_srv_channel))

        ## # get odom 2 base
        ## qos_profile = QoSProfile(
        ##     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,           
        ##     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        ##     depth=10
        ## )
        
        ## self.tf_buffer   = Buffer()
        ## self.tf_listener = TransformListener(buffer=self.tf_buffer,
        ##                                      node=self.node,
        ##                                      qos=qos_profile,
        ##                                          )
        
        self.feedback_message = "{}: finished setting up".format(self.name)
        return True


    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        ## if self.setup_flag is False: self.setup()
        
        self.sent_goal = False

        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key=self.name +'/grasp_pose', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=self.name +'/grasp_top_pose', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=self.name +'/place_pose', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key=self.name +'/place_top_pose', access=py_trees.common.Access.WRITE)
        
        self.blackboard.set(self.name +'/grasp_pose', Pose())
        self.blackboard.set(self.name +'/grasp_top_pose', Pose())
        self.blackboard.set(self.name +'/place_pose', Pose())    
        self.blackboard.set(self.name +'/place_top_pose', Pose())


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if not self.sent_goal:
            
            # Request the top surface pose of an object to WM
            obj = self.object_dict['target']
            req = StringPose.Request(data=obj)
            
            try:
                future = self.grasp_pose_srv_req.call_async(req)

                while rclpy.ok():
                    if future.done():
                        break
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                    ## time.sleep(0.05)
                
                obj_pose = future.result().pose
            except Exception as e:
                self.feedback_message = "Pose Service is not available: %s"%e
                return py_trees.common.Status.FAILURE

            try:
                future = self.height_srv_req.call_async(req)

                while rclpy.ok():
                    if future.done():
                        break
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                    ## time.sleep(0.05)

                obj_height = future.result().pose                
                obj_height = obj_height.position.z
            except Exception as e:
                self.feedback_message = "Height Service is not available: %s"%e
                return py_trees.common.Status.FAILURE

            future = self.tf_buffer.wait_for_transform_async(self._world_frame,
                                                        self._arm_base_frame,
                                                        rclpy.time.Time())
            #rclpy.spin_until_future_complete(self.tf_buffer, r)
            while rclpy.ok():
                if future.done(): break
                rclpy.spin_once(self.node, timeout_sec=0.5)
            
            pos = None
            while rclpy.ok():
                try:
                    t = self.tf_buffer.lookup_transform(self._world_frame,
                                                               self._arm_base_frame,
                                                               rclpy.time.Time())
                except TransformException as ex:
                    self.feedback_message = "WorldModel: Exception from TF"
                    continue
                if t is not None: break
                rclpy.spin_once(self.node, timeout_sec=0.5)
            pos = t.transform.translation
            quat = t.transform.rotation
                    
            self.base2arm_baselink = PyKDL.Frame(
                PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w),
                PyKDL.Vector(pos.x, pos.y, pos.z))

            # ------------ Compute -------------------------
            grasp_pose =\
            POSE_ESTIMATOR.get_grasp_pose(obj_pose, \
                                          self.base2arm_baselink, \
                                          self.grasp_offset_z)
            grasp_top_pose = copy.deepcopy(grasp_pose)
            grasp_top_pose.position.z += self.top_offset_z

            self.blackboard.set(self.name +'/grasp_pose', grasp_pose)
            self.blackboard.set(self.name +'/grasp_top_pose', grasp_top_pose)

            # Place pose
            if 'destination' in self.object_dict.keys():
                self.feedback_message = "getting the destination pose"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                destination = self.object_dict['destination']
                # Find a location on the destination top surface
                if self.en_random:
                    req      = StringPose.Request(data=json.dumps(self.object_dict))
                    future = self.rnd_pose_srv_req.call_async(req)
                elif self.en_close_pose:
                    req      = StringPose.Request(data=json.dumps(self.object_dict))
                    future = self.close_pose_srv_req.call_async(req)
                else:
                    req      = StringPose.Request(data=destination)
                    future = self.pose_srv_req.call_async(req)

                while rclpy.ok():
                    if future.done():
                        break
                    rclpy.spin_once(self.node, timeout_sec=0.05)
                    #TODO add timeout
                    ## self.node.get_logger().error( "Pose Service is not available: %s"%e )
                    ## self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                    ## return py_trees.common.Status.FAILURE
                    
                dst_pose = future.result().pose

                if 'destination_offset' in self.object_dict.keys():
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

        
        self.feedback_message = "WorldModel: successful grasp pose estimation "
        return py_trees.common.Status.SUCCESS
            
    
    def terminate(self, new_status):
        ## self.destroy_service(self.pose_srv_req)
        ## self.destroy_service(self.grasp_pose_srv_req)
        ## self.destroy_service(self.height_srv_req)
        ## self.destroy_service(self.rnd_pose_srv_req)
        ## self.destroy_service(self.close_pose_srv_req)
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
