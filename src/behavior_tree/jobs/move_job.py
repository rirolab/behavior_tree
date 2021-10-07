
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

#sys.path.insert(0,'..')
from behavior_tree.subtrees import MoveJoint, MovePose, Gripper, Stop, WorldModel


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
        self._grounding_channel = "symbol_grounding" #rospy.get_param('grounding_channel')
        
        ## self._subscriber = rospy.Subscriber("/dashboard/move", std_msgs.Empty, self.incoming)
        self._subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
        self._goal = None
        self._lock = threading.Lock()
        
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.gripper_open_pos = rospy.get_param("gripper_open_pos")
        self.blackboard.gripper_close_pos = rospy.get_param("gripper_close_pos")
        self.blackboard.init_config = eval(rospy.get_param("init_config", [0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.]))

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
                if grounding[str(i+1)]['primitive_action'] in ['move']:
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
        blackboard = py_trees.blackboard.Blackboard()
        
        if goal[idx]["primitive_action"] in ['move']:
            obj         = goal[idx]['object'].encode('ascii','ignore')
            destination = goal[idx]['destination'].encode('ascii','ignore')
        else:
            return None

        # TODO
        if destination=='na':
            destination='place_tray'
            print "destination is not assigned, so selected place-tray as a destination"
        
        # ----------------- Move Task ----------------        
        s_init3 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)

        # ----------------- Pick ---------------------
        pose_est1 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj})
        s_move10 = MovePose.MOVEPROOT(name="Top1",
                                      controller_ns=controller_ns,
                                      action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move11 = MovePose.MOVEP(name="Top2", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move12 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                               action_goal=blackboard.gripper_open_pos)        
        s_move13 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_pose"})
        s_move14 = Gripper.GOTO(name="Close", controller_ns=controller_ns,
                               action_goal=blackboard.gripper_close_pos)        
        s_move15 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})

        pick = py_trees.composites.Sequence(name="MovePick")
        pick.add_children([pose_est1, s_move10, s_move11, s_move12, s_move13, s_move14, s_move15])


        # ----------------- Place ---------------------
        place = py_trees.composites.Sequence(name="MovePlace")
        pose_est2 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj,
                                                             'destination': destination})
        s_move20 = MovePose.MOVEPROOT(name="Top1",
                                      controller_ns=controller_ns,
                                      action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move21 = MovePose.MOVEP(name="Top2", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move22 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_pose"})
        s_move23 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                               action_goal=50)        
        s_move24 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        
        place.add_children([pose_est2, s_move20, s_move21, s_move22, s_move23, s_move24, s_init3])
        task = py_trees.composites.Sequence(name="Move")
        task.add_children([pick, place])
        return task


