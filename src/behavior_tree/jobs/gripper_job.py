
import copy, sys
import move_base_msgs.msg as move_base_msgs
import py_trees, py_trees_ros
import rospy
import threading
import numpy as np
import json
import PyKDL

import std_msgs.msg as std_msgs
from geometry_msgs.msg import Pose, Quaternion #PointStamped,
from complex_action_client import misc

sys.path.insert(0,'..')
from subtrees import Gripper, Rosbag


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
        
        self._subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
        self._goal = None
        self._lock = threading.Lock()

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.gripper_open_pos = rospy.get_param("gripper_open_pos")
        self.blackboard.gripper_close_pos = rospy.get_param("gripper_close_pos")
        

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
            rospy.logerr("Gripper: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range( len(grounding.keys()) ):
                if grounding[str(i+1)]['primitive_action'].find('gripper_open')>=0 or\
                    grounding[str(i+1)]['primitive_action'].find('gripper_close')>=0:
                    self.goal = grounding #[str(i+1)]
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
    def create_root(idx="1", goal=std_msgs.Empty(), controller_ns="", rec_topic_list=None):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        if not ( goal[idx]["primitive_action"] in ['gripper_close', 'gripper_open'] ):
            return None
        
        # beahviors
        ## root = py_trees.composites.Sequence(name="Gripper")
        root = py_trees.composites.Parallel(name="Gripper",\
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        action = goal[idx]['primitive_action'].encode('ascii','ignore')

        blackboard = py_trees.blackboard.Blackboard()
        if action.find('gripper_close')>=0:        
            s_move = Gripper.GOTO(name="Close", action_goal=blackboard.gripper_close_pos)        
        else:
            s_move = Gripper.GOTO(name="Open", action_goal=blackboard.gripper_open_pos)        
            
        if rec_topic_list is None:
            root.add_child(s_move)
        else:
            logger = Rosbag.ROSBAG(name="logger", topic_list=rec_topic_list)
            root.add_children([s_move, logger])

        return root
