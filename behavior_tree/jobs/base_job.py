import copy, sys
import py_trees, py_trees_ros
import rclpy
import threading
import json

import std_msgs.msg as std_msgs

##############################################################################
# Behaviours
##############################################################################


class BaseJob(object):
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

        self.blackboard = py_trees.blackboard.Client()
        
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
            self._node.get_logger().error("JOB: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range( len(grounding.keys()) ):
                self.goal = grounding
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
        return None

