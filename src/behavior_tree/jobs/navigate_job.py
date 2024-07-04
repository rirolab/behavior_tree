
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
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose

sys.path.insert(0,'..')
from subtrees import MoveJoint, MovePose, MoveBase, MoveGoal, Gripper, Stop, WorldModel


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
                if grounding[str(i+1)]['primitive_action'] in ['move_to_goal']:
                    self.goal = grounding
                    break

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
        root = py_trees.composites.Sequence(name="navigate_job"+idx)
        blackboard = py_trees.blackboard.Blackboard()
        
        # move to goal
        if goal[idx]["primitive_action"] in ['move_to_goal']:
            if 'goal' in goal[idx].keys():
                goal = goal[idx]['destination']
            else:
                rospy.logerr("No navigation goal")
                sys.exit()
        else:
            return None

        # ----------------- Navigate ---------------------
        navigate = py_trees.composites.Sequence(name="navigate")
        s_drive = MoveGoal.MOVEG(name = "Drive", idx=idx, destination=goal,
                                 action_goal={'pose': goal})
        navigate.add_children([s_drive])

        root.add_children([navigate])
        return root
