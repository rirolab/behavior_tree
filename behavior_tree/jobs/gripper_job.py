import copy, sys
import py_trees, py_trees_ros
import rclpy
import threading
import json

import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.validation_utils import StepValidationResult
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
        self.init_blackboard_parameters()

    def acceptable_step(self, step):
        """
        Check whether this job should accept a grounding step for this primitive action.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        # Check if the primitive action is gripper_open or gripper_close
        if step.get("primitive_action") not in ["gripper_close", "gripper_open"]:
            return False

        # Check if the step has the number of robots required for this job
        elif not self.check_robot_count(step, num_robot_required=1):
            return False

        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable gripper step is well-formed enough to
        keep the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        # Check if the step has the required parameters for gripper commands
        if self.acceptable_step(step):
            return StepValidationResult.ACCEPT_GOAL
        else:
            return StepValidationResult.NOT_APPLICABLE

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
            for i in range(len(grounding.keys())):
                step = grounding.get(str(i + 1))
                if step is None:
                    continue
                if self.acceptable_step(step):
                    self.goal = grounding
                    break

    def create_root(
        self,
        action_client,
        idx="1",
        goal=std_msgs.Empty(),
        rec_topic_list=None,
        robot_name=None,
        **kwargs,
    ):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        # Check if the step is acceptable
        if not self.acceptable_step(goal[idx]):
            return None
        
        # beahviors
        ## root = py_trees.composites.Parallel(name="Gripper",\
        ##                                     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        root = py_trees.composites.Sequence(name="Gripper", memory=True)
        action = goal[idx]['primitive_action']

        blackboard = py_trees.blackboard.Client(namespace=robot_name)
        blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)

        if action.find('gripper_close')>=0:        
            s_move = Gripper.GOTO(name="Close", action_client=action_client,
                                action_goal=blackboard.gripper_close_pos,
                                force=blackboard.gripper_close_force,
                                robot_name=robot_name)
        else:
            s_move = Gripper.GOTO(name="Open", action_client=action_client,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force,
                                robot_name=robot_name)
            
        if rec_topic_list is None:
            root.add_child(s_move)
        ## else:
        ##     logger = Rosbag.ROSBAG(name="logger", topic_list=rec_topic_list)
        ##     root.add_children([s_move, logger])

        return root

    
