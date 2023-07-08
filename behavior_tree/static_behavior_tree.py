#!/usr/bin/env python3
import sys
import numpy as np
import argparse
import operator

import rclpy
import rclpy.node
from rclpy.parameter import Parameter

import functools
import py_trees
import py_trees_ros
import py_trees.console as console

from .subtrees import WM2Blackboard
from .subtrees import MoveJoint

import time
from riro_srvs.srv import StringInt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from py_trees_ros import exceptions, utilities

def create_root(node, controller_ns=""):
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.
    
    Returns:
    :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="StaticTree",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False )
        )

    # Shared services
    action_client = node.create_client(StringInt, 'arm_client/command',
                                 qos_profile=rclpy.qos.qos_profile_services_default)
    if not action_client.wait_for_service(timeout_sec=3.0):
        raise exceptions.TimedOutError('command service not available, waiting again...')
                

    # ---------------- Root->Blackboard ------------------------
    topics2bb = py_trees.composites.Sequence("Topics2BB", memory=True)
    wm2bb = WM2Blackboard.ToBlackboard(name="WM2BB",
                                       topic_name="/world_model",
                                       qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                       )
    cmd2bb = py_trees_ros.subscribers.EventToBlackboard(
        name="Cmd2BB",
        topic_name="/move_cmd",
        variable_name="move_cmd",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
        )

    root.add_child(topics2bb)
    topics2bb.add_children([cmd2bb,wm2bb])

    # ---------------- Root->Priorities- -----------------------
    priorities = py_trees.composites.Selector("Priorities", memory=False)
    idle = py_trees.behaviours.Running(name="Idle")
    root.add_child(priorities)

    # ---------------- Tasks -----------------------------------
    # Move
    seq_move = py_trees.composites.Sequence(name="Move", memory=True)
    is_move = py_trees.behaviours.CheckBlackboardVariableValue(
        name="Move?",
        check=py_trees.common.ComparisonExpression(
            variable="move_cmd",
            value=True,
            operator=operator.eq
            )
        )
    
    s_move1 = MoveJoint.MOVEJ(name="Move1", action_client=action_client,
                              controller_ns=controller_ns,
                              action_goal=[0, -np.pi/2.0, np.pi/2.0, 0, np.pi/2., np.pi/2.])
    s_move2 = MoveJoint.MOVEJ(name="Move2", action_client=action_client,
                              controller_ns=controller_ns,
                              action_goal=[0, -np.pi/2.0, np.pi/2.0, 0, np.pi/2., -np.pi/2.])
    seq_move.add_children([is_move, s_move1, s_move2])
                            
    priorities.add_children([seq_move, idle]) 
    return root

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

    
##############################################################################
# Main
##############################################################################
def main(args=None):
    """
    Entry point for the demo script.
    """

    args = get_args(sysargv=sys.argv)[0]

    rclpy.init()        
    node = rclpy.create_node("tree")

    node.declare_parameters(
        namespace='',
        parameters=[
            ('controller_ns', Parameter.Type.STRING),
            ]
    )    
    controller_ns=node.get_parameter("controller_ns").get_parameter_value().string_value
    
    root = create_root(node, controller_ns)
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root=root,
                                                      unicode_tree_debug=True)

    try:
        behaviour_tree.setup(node=node, timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        node.get_logger().error("failed to setup the tree, aborting [{}]".format(str(e)))
        behaviour_tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        node.get_logger().error("tree setup interrupted")
        behaviour_tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)

    ## #executor = rclpy.executors.MultiThreadedExecutor()    
    ## #rclpy.spin(behaviour_tree, executor)
    ## executor = rclpy.executors.SingleThreadedExecutor()
    ## executor.add_node(behaviour_tree)

    ## number_of_iterations = 10000
    ## behaviour_tree.tick_tock(period_ms=500, number_of_iterations=number_of_iterations)
    ## node.get_logger().warn("-------------------------- {} : {}".format(root.status, py_trees.common.Status.RUNNING))
    ## while behaviour_tree.count < number_of_iterations and root.status == py_trees.common.Status.RUNNING:
    ##     node.get_logger().warn("{}".format(behaviour_tree.count))
    ##     executor.spin_once(timeout_sec=0.05)

    behaviour_tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(behaviour_tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        behaviour_tree.shutdown()
        rclpy.try_shutdown()

    
def get_args(sysargv=None):
    p = argparse.ArgumentParser(
        description="Run a behavior tree")
    p.add_argument('--viz', action='store_true', dest='viz',
                 default=False,
                 help='use visualization code for rviz')
    p.add_argument('--rec_topics', action='store', dest='topic_json',
                 default=None, help='a list of topic to record')
    argv = sysargv[1:] if sysargv is not None else sys.argv[1:]
    args = p.parse_known_args(argv)
    return args

    
if __name__ == '__main__':
    main()
