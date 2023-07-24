import copy, sys
import py_trees, py_trees_ros
import rclpy
import threading
import numpy as np
import json
import PyKDL

import std_msgs.msg as std_msgs
from complex_action_client import misc

from behavior_tree.subtrees import MoveJoint, MovePose, Gripper, Stop, WorldModel


##############################################################################
# Behaviours
##############################################################################


class Move(object):
    """
    A job handler that instantiates a subtree for scanning to be executed by
    a behaviour tree.
    """

    def __init__(self, node):
        """
        Tune into a channel for incoming goal requests. This is a simple
        subscriber here but more typically would be a service or action interface.
        """
        self._node = node
        self._grounding_channel = "symbol_grounding" 
        
        self._subscriber = self._node.create_subscription(std_msgs.String, \
                                                          self._grounding_channel,
                                                          self.incoming, 10)
        self._goal = None
        self._lock = threading.Lock()
        
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.gripper_open_pos    = self._node.get_parameter_or("gripper_open_pos", 0.0).get_parameter_value().double_value
        self.blackboard.gripper_close_pos   = self._node.get_parameter_or("gripper_close_pos", 0.8).get_parameter_value().double_value
        self.blackboard.gripper_open_force  = self._node.get_parameter_or("gripper_open_force", 1.).get_parameter_value().double_value
        self.blackboard.gripper_close_force = self._node.get_parameter_or("gripper_close_force", 1.).get_parameter_value().double_value
        self.blackboard.init_config = self._node.get_parameter("init_config").value
        if type(self.blackboard.init_config) is str:
            self.blackboard.init_config = eval(self.blackboard.init_config)

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
            self._node.get_logger().error("handover_job: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range( len(grounding.keys()) ):
                if grounding[str(i+1)]['primitive_action'] in ['handover']:
                    self.goal = grounding #[str(i+1)] )
                    break

    @staticmethod
    def create_root(node, action_client, idx="1", goal=std_msgs.Empty(), **kwargs):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        # beahviors
        root = py_trees.composites.Sequence(name="Handover")
        blackboard = py_trees.blackboard.Blackboard()

        if goal[idx]["primitive_action"] in ['handover']:
            obj = goal[idx]['object'].encode('ascii','ignore')
            destination = goal[idx]['destination'].encode('ascii','ignore')
        else:
            return None

        # TODO
        if destination=='na':
            destination = None
        
        # ----------------- Init Task ----------------        
        s_init = MoveJoint.MOVEJ(name="Init", action_client=action_client,
                                  action_goal=blackboard.init_config)

        # ----------------- Handover ---------------------
        s_move1 = MoveJoint.MOVEJ(name="Front", action_client=action_client,
                                  action_goal=[np.pi/2, -np.pi/2., np.pi/2., np.pi/2., -np.pi/2., 0])
        s_move2 = Gripper.GOTO(name="Open", action_client=action_client,
                                   action_goal=blackboard.gripper_open_pos, check_contact=True)        

        wm_remove = WorldModel.REMOVE(name="Delete", action_goal={'obj_name': obj})


        task = py_trees.composites.Sequence(name="Handover")
        task.add_children([s_move1, s_move2, s_init, wm_remove])
        return task

