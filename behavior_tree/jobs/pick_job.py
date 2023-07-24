import copy, sys
import threading
import json
import rclpy

import py_trees, py_trees_ros
import py_trees.console as console

import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MoveJoint, MovePose, Gripper, WorldModel


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
        self.blackboard.init_config = self._node.get_parameter("init_config").value
        if type(self.blackboard.init_config) is str:
            self.blackboard.init_config = eval(self.blackboard.init_config)

            
    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            console.logerror("MOVE: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range( len(grounding.keys()) ):
                if grounding[str(i+1)]['primitive_action'] in ['pick']:
                    self.goal = grounding #[str(i+1)] )
                    break
                
    @staticmethod
    def create_root(action_client, idx="1", goal=std_msgs.Empty(), **kwargs):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        # beahviors
        root = py_trees.composites.Sequence(name="Pick", memory=True)
        blackboard = py_trees.blackboard.Client()
        blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)

        if goal[idx]["primitive_action"] in ['pick']:
            if 'object' in goal[idx].keys():
                obj = goal[idx]['object']
            elif 'obj' in goal[idx].keys():
                obj = goal[idx]['obj']
            else:
                console.logerror("Pick: No pick object")
                sys.exit()                
        else:
            return None
        
        # ------------ Compute -------------------------
        s_init1 = MoveJoint.MOVEJ(name="Init",\
                                  action_client=action_client,\
                                  action_goal=blackboard.init_config)
        s_init2 = MoveJoint.MOVEJ(name="Init2",\
                                  action_client=action_client,\
                                  action_goal=blackboard.init_config)

        # ----------------- Pick ---------------------
        pose_est1 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj},
                                              tf_buffer=kwargs['tf_buffer'])
        s_move11 = MovePose.MOVEP(name="Top",\
                                  action_client=action_client,\
                                  action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move12 = Gripper.GOTO(name="Open",
                                action_client=action_client,\
                                action_goal=blackboard.gripper_open_pos,\
                                force=blackboard.gripper_open_force)        
        s_move13 = MovePose.MOVEP(name="Approach",\
                                  action_client=action_client,\
                                  action_goal={'pose': "Plan"+idx+"/grasp_pose"})
        s_move14 = Gripper.GOTO(name="Close",\
                                action_client=action_client,\
                                action_goal=blackboard.gripper_close_pos,\
                                force=blackboard.gripper_close_force)        
        s_move15 = MovePose.MOVEP(name="Top",\
                                  action_client=action_client,\
                                  action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})

        pick = py_trees.composites.Sequence(name="Pick", memory=True)
        pick.add_children([pose_est1, s_init1, s_move11, s_move12, s_move13, s_move14, s_move15, s_init2])
        return pick

    
