import copy, sys
import py_trees, py_trees_ros
import rclpy
import threading
import json

import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import Gripper #, Rosbag


##############################################################################
# Behaviours
##############################################################################

class Move(base_job.BaseJob):
    """
    A job handler that instantiates a subtree for scanning to be executed by
    a behaviour tree.
    """

    def __init__(self, node):
        """
        Tune into a channel for incoming goal requests. This is a simple
        subscriber here but more typically would be a service or action interface.
        """
        super(Move, self).__init__(node)
        
        self.blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="init_config", access=py_trees.common.Access.WRITE)
        self.blackboard.gripper_open_pos    = self._node.get_parameter("gripper_open_pos").get_parameter_value().double_value
        self.blackboard.gripper_close_pos   = self._node.get_parameter("gripper_close_pos").get_parameter_value().double_value
        self.blackboard.gripper_open_force  = self._node.get_parameter("gripper_open_force").get_parameter_value().double_value
        self.blackboard.gripper_close_force = self._node.get_parameter("gripper_close_force").get_parameter_value().double_value
        

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            self._node.get_logger().error("gripper_job: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range( len(grounding.keys()) ):
                if grounding[str(i+1)]['primitive_action'].find('gripper_open')>=0 or\
                    grounding[str(i+1)]['primitive_action'].find('gripper_close')>=0:
                    self.goal = grounding #[str(i+1)]
                    break


    @staticmethod
    def create_root(action_client, idx="1", goal=std_msgs.Empty(), rec_topic_list=None, **kwargs):
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
        ## root = py_trees.composites.Parallel(name="Gripper",\
        ##                                     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        root = py_trees.composites.Sequence(name="Gripper", memory=True)
        action = goal[idx]['primitive_action']

        blackboard = py_trees.blackboard.Client()
        blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)

        if action.find('gripper_close')>=0:        
            s_move = Gripper.GOTO(name="Close", action_client=action_client,
                                action_goal=blackboard.gripper_close_pos,
                                force=blackboard.gripper_close_force)        
        else:
            s_move = Gripper.GOTO(name="Open", action_client=action_client,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force)        
            
        if rec_topic_list is None:
            root.add_child(s_move)
        ## else:
        ##     logger = Rosbag.ROSBAG(name="logger", topic_list=rec_topic_list)
        ##     root.add_children([s_move, logger])

        return root

    
