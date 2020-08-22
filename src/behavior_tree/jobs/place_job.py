
import copy, sys
import move_base_msgs.msg as move_base_msgs
import py_trees, py_trees_ros
import rospy
import threading
import numpy as np
import json
import PyKDL
import tf

import std_msgs.msg as std_msgs
from complex_action_client import misc

sys.path.insert(0,'..')
from subtrees import MoveJoint, MovePose, Gripper, Stop, WorldModel


##############################################################################
# Behaviours
##############################################################################


class Move(object):
    """
    A job handler that instantiates a subtree for scanning to be executed by
    a behaviour tree.
    """

    def __init__(self):
        """
        Tune into a channel for incoming goal requests. This is a simple
        subscriber here but more typically would be a service or action interface.
        """
        self._grounding_channel = "/symbol_grounding" #rospy.get_param('grounding_channel')
        
        self._subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
        self._goal = None
        self._lock = threading.Lock()
        ## self.blackboard = py_trees.blackboard.Blackboard()

        ## self.object      = None
        ## self.destination = None

    @property
    def goal(self):
        """
        Getter for the variable indicating whether or not a goal has recently been received
        but not yet handled. It simply makes sure it is wrapped with the appropriate locking.
        """
        with self._lock:
            g = copy.copy(self._goal) or self._goal
        return g

    @goal.setter
    def goal(self, value):
        """
        Setter for the variable indicating whether or not a goal has recently been received
        but not yet handled. It simply makes sure it is wrapped with the appropriate locking.
        """
        with self._lock:
            self._goal = value

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            rospy.logerr("MOVE: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range( len(grounding.keys()) ):
                if grounding[str(i+1)]['primitive_action'] in ['place']:
                    self.goal = grounding #[str(i+1)] )
                    break
                

            
    ## def create_report_string(self, subtree_root):
    ##     """
    ##     Introspect the subtree root to determine an appropriate human readable status report string.

    ##     Args:
    ##         subtree_root (:class:`~py_trees.behaviour.Behaviour`): introspect the subtree

    ##     Returns:
    ##         :obj:`str`: human readable substring
    ##     """
    ##     if subtree_root.tip().has_parent_with_name("Cancelling?"):
    ##         return "cancelling"
    ##     else:
    ##         return "scanning"


    @staticmethod
    def create_root(idx="1", goal=std_msgs.Empty(), controller_ns="", **kwargs):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """       
        # beahviors
        root = py_trees.composites.Sequence(name="Move")
        grasp_offset_z = 0.02
        
        if goal[idx]["primitive_action"] in ['place']:
            if 'object' in goal[idx].keys():
                obj = goal[idx]['object'].encode('ascii','ignore')
            elif 'obj' in goal[idx].keys():
                obj = goal[idx]['obj'].encode('ascii','ignore')
            else:
                rospy.logerr("MOVE: No place object")
                sys.exit()
                
            destination = goal[idx]['destination'].encode('ascii','ignore')
            destination_offset = goal[idx]['destination_offset'] #.encode('ascii','ignore')
            
            ## target_frame = goal[idx]['base'].encode('ascii','ignore')
            ## target_pose  = goal[idx]['target']
            ## tgt2obj = PyKDL.Frame(PyKDL.Rotation.RotZ(target_pose[3]),
            ##                       PyKDL.Vector(target_pose[0],
            ##                                    target_pose[1],
            ##                                    target_pose[2]))
        else:
            return None

        ## # Request the top surface pose of an object to WM
        ## height_srv_channel = 'get_object_height'
        ## rospy.wait_for_service(height_srv_channel)
        ## try:
        ##     srv_req = rospy.ServiceProxy(height_srv_channel, String_Pose)
        ##     obj_height = srv_req(obj).pose
        ## except rospy.ServiceException, e:
        ##     print "Height Service is not available: %s"%e
        ##     sys.exit()
        ## obj_height = obj_height.position.z

        ## # get odom 2 base
        ## arm_base_frame_id = rospy.get_param("arm_base_frame", '/ur_arm_base_link')
        ## listener = tf.TransformListener()
        ## pos = None
        ## while not rospy.is_shutdown():
        ##     try:
        ##         (pos,quat) = listener.lookupTransform(target_frame, arm_base_frame_id, rospy.Time(0))
        ##     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        ##         ## rospy.loginfo("{} frame name is not available.".format(target_frame))
        ##         continue
        ##     if pos is not None: break

        ## tgt2arm_baselink = PyKDL.Frame(
        ##     PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3]),
        ##     PyKDL.Vector(pos[0], pos[1], pos[2]))                
        
        # ------------ Compute -------------------------
        ## arm_baselink2obj  = tgt2arm_baselink.Inverse() * tgt2obj
        
        ## obj2grasp      = PyKDL.Frame(PyKDL.Rotation.RotX(-np.pi/2.) * PyKDL.Rotation.RotY(-np.pi/2.))
        ## baselink2grasp = arm_baselink2obj * obj2grasp
        
        ## arm_baselink2obj = tgt2arm_baselink.Inverse() * tgt2obj
        ## arm_baselink2obj.M = baselink2grasp.M
        
        ## place_pose     = misc.KDLframe2Pose(arm_baselink2obj)
        ## place_pose.position.z += obj_height
        ## place_pose.position.z -= grasp_offset_z
        ## place_top_pose = copy.deepcopy(place_pose)
        ## place_top_pose.position.z += 0.15
            
        
        
        s_init3 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=[0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.])

        # ----------------- Place ---------------------
        place = py_trees.composites.Sequence(name="Place")
        pose_est2 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj,
                                                             'destination': destination,
                                                             'destination_offset': destination_offset})
        #from IPython import embed; embed(); sys.exit()
        s_move21 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move22 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_pose"})
        s_move23 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                               action_goal=50)        
        s_move24 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        
        place.add_children([pose_est2, s_move21, s_move22, s_move23, s_move24, s_init3])
        return place


