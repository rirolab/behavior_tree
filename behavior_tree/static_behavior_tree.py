#!/usr/bin/env python3
import sys
import numpy as np
import argparse

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import functools
import py_trees
import py_trees_ros
import py_trees.console as console

import WM2Blackboard
import MoveJoint


def create_root(controller_ns=""):
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.
    
    Returns:
    :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = py_trees.composites.Parallel("Tutorial")

    # ---------------- Root->Blackboard ------------------------
    topics2bb = py_trees.composites.Sequence("Topics2BB")
    wm2bb = WM2Blackboard.ToBlackboard(name="WM2BB",
                                       topic_name="/world_model" )
    cmd2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cmd2BB",
        topic_name="/move_cmd",
        variable_name="move_cmd"
        )

    root.add_child(topics2bb)
    topics2bb.add_children([cmd2bb,wm2bb])

    # ---------------- Root->Priorities- -----------------------
    priorities = py_trees.composites.Selector("Priorities")
    idle = py_trees.behaviours.Running(name="Idle")
    root.add_child(priorities)

    # ---------------- Tasks -----------------------------------
    # Move
    seq_move = py_trees.composites.Sequence(name="Move")
    is_move = py_trees.blackboard.CheckBlackboardVariable(
        name="Move?",
        variable_name="move_cmd",
        expected_value=True
        )
    
    s_move1 = MoveJoint.MOVEJ(name="Move1", controller_ns=controller_ns,
                              action_goal=[0, -np.pi/2.0, np.pi/2.0, 0, np.pi/2., -np.pi/4.])
    s_move2 = MoveJoint.MOVEJ(name="Move2", controller_ns=controller_ns,
                              action_goal=[0, -np.pi/2.0, np.pi/2.0, 0, np.pi/2., np.pi/4.])
    seq_move.add_children([is_move, s_move1, s_move2])
                            
    priorities.add_children([seq_move,idle])
    return root

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

    
##############################################################################
# Main
##############################################################################
def main(argv=sys.argv):
    """
    Entry point for the demo script.
    """

    args = get_args(sysargv=argv)

    rclpy.init()        
    node = rclpy.create_node("behavior_tree")

    node.declare_parameters(
        namespace='',
        parameters=[
            ('controller_ns', Parameter.Type.STRING),
            ]
    )
    
    root = create_root(node.get_parameter("controller_ns").value)
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rclpy.get_default_context().on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(500)
    
    node.destroy_node()
    rclpy.shutdown()

    
def get_args(sysargv=None):
    p = argparse.ArgumentParser(
        description="Run a behavior tree")
    p.add_argument('--viz', action='store_true', dest='viz',
                 default=False,
                 help='use visualization code for rviz')
    p.add_argument('--rec_topics', action='store', dest='topic_json',
                 default=None, help='a list of topic to record')
    argv = sysargv[1:] if sysargv is not None else sys.argv[1:]
    args = p.parse_args(argv)
    return args

    
if __name__ == '__main__':
    main()
