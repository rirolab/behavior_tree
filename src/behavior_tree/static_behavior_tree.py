#!/usr/bin/env python
# Copyright YYYY Massachusetts Institute of Technology 

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys
import numpy as np

import WM2Blackboard
import MoveJoint


# 


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
    ## task = py_trees.meta.success_is_failure(py_trees.composites.Selector)(name="Task")
    root.add_child(priorities)

    # ---------------- Tasks -----------------------------------
    # Move
    seq_move = py_trees.composites.Sequence(name="Move")
    is_move = py_trees.blackboard.CheckBlackboardVariable(
        name="Move?",
        variable_name="move_cmd",
        expected_value=True
        )
    ## move_preempt = py_trees.composites.Sequence(name="Preempt?")
    ## is_move_two  = py_trees.meta.success_is_running(py_trees.blackboard.CheckBlackboardVariable)(
    ##     name="Move?",
    ##     variable_name="move_cmd",
    ##     expected_value=True
    ##     )
    
    s_move1 = MoveJoint.MOVEJ(name="Move1", controller_ns=controller_ns,
                              action_goal=[0, -np.pi/2.0, np.pi/2.0, 0, np.pi/2., -np.pi/4.])
    s_move2 = MoveJoint.MOVEJ(name="Move2", controller_ns=controller_ns,
                              action_goal=[0, -np.pi/2.0, np.pi/2.0, 0, np.pi/2., np.pi/4.])
    seq_move.add_children([is_move, s_move1, s_move2])
    ## seq_move.add_children([is_move, move_preempt])
    ## move_preempt.add_children([s_move1, s_move2])
    #move_preempt.add_children([is_move_two, s_move1, s_move2])
    
    ## # Pick
    ## t_pick = py_trees.composites.Sequence(name="Pick")

    ## # Handover
    ## t_handover = py_trees.composites.Sequence(name="Handover")
    ## mv2  = MoveJoint.MOVEJ(name="Move2")
                            
    priorities.add_children([seq_move,idle])
    return root

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    """
    Entry point for the demo script.
    """
    import optparse
    p = optparse.OptionParser()
    p.add_option('--sim', action='store_true', dest='sim',
                 help='use this option if you use simulated ur5')
    p.add_option('--viz', '-v', action='store_true', dest='viz',
                 help='use visualization code for rviz')
    opt, args = p.parse_args()

    if opt.sim:
        controller_ns = "arm_controller"
    else:
        controller_ns = ""

    
    rospy.init_node("tree")
    root = create_root(controller_ns)
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(500)
    
    
