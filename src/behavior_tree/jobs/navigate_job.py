
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
# from complex_action_client import misc
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose

sys.path.insert(0,'..')
from subtrees import MoveGoal


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
        self._name = "waypoint_navigation"
        # self._grounding_channel = "symbol_grounding" #rospy.get_param('grounding_channel')
        # self._subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
        self._goal = None
        self._lock = threading.Lock()

    @property
    def name(self):
        return self._name
    @name.getter
    def name(self):
        return self._name 

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
                if grounding[str(i+1)]['primitive_action'] in ['waypoint_navigation']:
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
        root = py_trees.composites.Sequence(name="waypoint_navigation"+idx)
        blackboard = py_trees.blackboard.Blackboard()
        waypoints = None
        
        # move to goal
        if goal["primitive_action"] in ['waypoint_navigation']:
            # if 'goal' in goal[idx].keys():
            #     goal = goal[idx]['destination']
            if goal['destination'] is not "na":
                # string, goal['destination']: r1, r2, ...
                # These are transition states
                waypoints = goal['destination'] 
            else:
                rospy.logerr("No navigation goal")
                sys.exit()
        else:
            return None

        # ----------------- Navigate ---------------------
        # name: navigate_job1, navigate_job2, ...
        # navigate_job = py_trees.composites.Sequence(name='move')

        # Configure the subtree for the navigate_job
        root_children = []
        rospy.logerr(f"WORLD_MODEL: {blackboard.wm_dict}")
        rospy.logerr(f"WAYPOINTS: {waypoints}")
        if waypoints is not None:
            for wp_idx, waypoint in enumerate(waypoints):
                for _, v in blackboard.wm_dict.items():
                    if v['name'] == waypoint:
                        pose = v['location']
                        break
                ##### tried to discriminate between relax and move, but it didn't work ############
                # parallel_block = py_trees.composites.Parallel(name=f"gotopoint{wp_idx+1}",
                #                                               policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,) # TODO : change name?
                # parallel_block.synchronise = False
                
                # relax = MoveGoal.RELAX(name = "relax?",
                #                        idx = wp_idx,
                #                        destination=waypoint,
                #                        action_goal = {'pose': pose},
                #                        waypoint_seq=waypoints[wp_idx:])
                # move = MoveGoal.MOVEG(name = "navigate",
                #                       idx = wp_idx, 
                #                       destination = waypoint,
                #                       action_goal = {'pose': pose},
                #                       waypoint_seq=waypoints[wp_idx:])
                
                # parallel_block.add_children([relax, move])
                # root_children.append(parallel_block)
                ######################################################################
                
                
                move = MoveGoal.MOVEG(name = f"gotopoint{wp_idx+1}",
                                      idx = wp_idx, 
                                      destination = waypoint,
                                      action_goal = {'pose': pose},
                                      waypoint_seq=waypoints[wp_idx:])
                root_children.append(move)
        root.add_children(root_children)

        return root
