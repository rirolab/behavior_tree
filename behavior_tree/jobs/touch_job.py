
import copy, sys
import threading
import numpy as np
import rospy

import py_trees
import py_trees_ros

import move_base_msgs.msg as move_base_msgs
import std_msgs.msg as std_msgs
from geometry_msgs.msg import Pose, Quaternion #PointStamped,

sys.path.insert(0,'..')
from subtrees import MoveJoint, MovePose, Gripper, Stop


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
        self._subscriber = rospy.Subscriber("/dashboard/test_move", std_msgs.Empty, self.incoming)
        self._goal = None
        self._lock = threading.Lock()

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.init_config = eval(rospy.get_param("init_config", [0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.]))
        
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
            rospy.logerr("TestMove: rejecting new goal, previous still in the pipeline")
        else:
            self.goal = msg

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
        if not ( goal[idx]["primitive_action"] in ['test'] ):
            return None
        
        # beahviors
        root  = py_trees.composites.Sequence(name="TestMove")
        frame = rospy.get_param("arm_base_frame", '/ur_arm_base')
        blackboard = py_trees.blackboard.Blackboard()
        
        s_move1 = MoveJoint.MOVEJ(name="Move1", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)


        pose = Pose()
        pose.position.z=-0.1
        goal_dict = {'pose': pose,
                     'frame': frame}
        
        s_move2 = MovePose.MOVEPR(name="Move2", controller_ns=controller_ns,
                                  action_goal=goal_dict)

        pose = Pose()
        pose.position.z=0.1
        goal_dict = {'pose': pose,
                     'frame': frame}
        s_move3 = MovePose.MOVEPR(name="Move3", controller_ns=controller_ns,
                                  action_goal=goal_dict)


        pose = Pose()
        pose.position.z=-0.1
        goal_dict = {'pose': pose,
                     'frame': frame}
        
        s_move4 = MovePose.MOVEPR(name="Move4", controller_ns=controller_ns,
                                  action_goal=goal_dict)

        pose = Pose()
        pose.position.z=0.1
        goal_dict = {'pose': pose,
                     'frame': frame}
        
        s_move5 = MovePose.MOVEPR(name="Move5", controller_ns=controller_ns,
                                  action_goal=goal_dict)
        
        root.add_children([s_move1, s_move3, s_move2]) #, s_move4, s_move5])

        return root
