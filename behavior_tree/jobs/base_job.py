import copy, sys
import py_trees, py_trees_ros
import rclpy
import threading
import json

import std_msgs.msg as std_msgs
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult

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

    def check_grounding(self, step, num_robot_required):
        """
        Check whether this job should accept a grounding step for the tree robots.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.
            num_robot_required (:obj:`int`): number of robots required for this job.

        Returns:
            :obj:`bool`: whether this job can handle the grounding step.
        """

        bt_robot_names = getattr(self._node, "robot_names", [])
        grounding_robot_names = make_string_list(step.get("robot"))

        # Case: single-tree node without configured robot names.
        # TODO: consider multi_dynamic_behavior_trees with no robot names
        if not bt_robot_names:
            return True

        # Case: one configured robot in BT
        if len(bt_robot_names) == 1:

            # Case: A missing robot field is accepted in grounding msg if a single robot is configured.
            # If the robot field is present, it must match the configured robot.
            if num_robot_required == 1:
                if not grounding_robot_names:
                    return True
                else:
                    return (
                        len(grounding_robot_names) == 1
                        and grounding_robot_names[0] == bt_robot_names[0]
                    )

            # Case: A multi-robot job requires multiple robots in BT.
            else:
                return False

        # Case: multiple configured robots in BT
        else:
            # Case: multi-robot tree requires the grounding step.
            if not grounding_robot_names:
                return False

            # Case: single-robot job in a multi-robot tree accepts one robot if it matches one of the configured robots.
            if num_robot_required == 1:
                return len(grounding_robot_names) == 1 and grounding_robot_names[0] in bt_robot_names

            # Case: multi-robot job in a multi-robot tree
            else:
                # Case: reject grounding if it contains robots not in the tree configuration.
                if not set(grounding_robot_names).issubset(set(bt_robot_names)):
                    return False

                # Case: multi-robot job requires the expected number of robots.
                return len(grounding_robot_names) == num_robot_required

    def check_robot_count(self, step, num_robot_required):
        """
        Check only the job-specific robot-count requirements for one grounding step.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can handle the grounding step shape.
        """
        bt_robot_names = getattr(self._node, "robot_names", [])
        grounding_robot_names = make_string_list(step.get("robot"))

        # Accept an omitted robot field only for single-robot jobs when the tree
        # is single-robot or does not declare robot names at all.
        if not grounding_robot_names:
            return num_robot_required == 1 and len(bt_robot_names) <= 1

        # Single-robot jobs accept exactly one requested robot.
        if num_robot_required == 1:
            return len(grounding_robot_names) == 1

        # Multi-robot jobs accept the expected number of requested robots.
        return len(grounding_robot_names) == num_robot_required

    def validate_step(self, step):
        """
        Validate whether one step should reject the whole goal for this job.

        Returns:
            :class:`StepValidationResult`: validation outcome for this job.
        """
        return StepValidationResult.NOT_APPLICABLE

    def acceptable_step(self, step):
        """
        Check whether this job can register one grounding step.

        Subclasses should override this to keep the incoming-goal filter and the
        subtree-build filter aligned.
        """
        return False

    def init_blackboard_parameters(self):
        """
        Write node parameters to the blackboard namespaces used by this job.
        """
        parameter_names = self._node.list_parameters([], 0).names
        robot_names = getattr(self._node, "robot_names", [])

        # Set up global parameters in the global blackboard namespace.
        global_blackboard = py_trees.blackboard.Client()
        global_parameter_keys = {
            parameter_name
            for parameter_name in parameter_names
            if "." not in parameter_name
        }
        for key in sorted(global_parameter_keys):
            global_blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
            global_blackboard.set(
                key,
                self._node.get_parameter(key).value,
            )

        # Set up parameters for each robot namespace.
        for robot_name in robot_names:
            blackboard = py_trees.blackboard.Client(namespace=robot_name)
            prefix = robot_name + "."
            parameter_keys = {
                parameter_name[len(prefix):]
                for parameter_name in parameter_names
                if parameter_name.startswith(prefix)
            }
            for key in sorted(parameter_keys):
                blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)
                blackboard.set(
                    key,
                    self._node.get_parameter(f"{robot_name}.{key}").value,
                )
        
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
            for i in range(len(grounding.keys())):
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
