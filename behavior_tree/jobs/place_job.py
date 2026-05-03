import copy, sys
import py_trees, py_trees_ros
import rclpy
import threading
import json

import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.validation_utils import StepValidationResult
from behavior_tree.subtrees import MoveJoint, MovePose, Gripper, Policy, WorldModel


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
        # Check if the primitive action is place
        if step.get("primitive_action") != "place":
            return False

        # Check if the step has the number of robots required for this job
        elif not self.check_robot_count(step, num_robot_required=1):
            return False

        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable place step is well-formed enough to keep
        the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        # Check if the step has the required parameters for place
        if self.acceptable_step(step):
            if step.get("implementation") == "policy":
                if bool(step.get("policy_name")):
                    return StepValidationResult.ACCEPT_GOAL
                else:
                    return StepValidationResult.REJECT_GOAL
            elif (("object" in step) or ("obj" in step)) and ("destination" in step):
                return StepValidationResult.ACCEPT_GOAL
            else:
                return StepValidationResult.REJECT_GOAL
        else:
            return StepValidationResult.NOT_APPLICABLE
        
    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            self._node.get_logger().error("place_job: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range(len(grounding.keys())):
                step = grounding.get(str(i + 1))
                if step is None:
                    continue
                if self.acceptable_step(step):
                    self.goal = grounding
                    break
                
    def create_root(self, action_client, idx="1", goal=std_msgs.Empty(), robot_name=None, **kwargs):
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
        root = py_trees.composites.Sequence(name="Place", memory=True)
        blackboard = py_trees.blackboard.Client(namespace=robot_name)
        blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)
        
        if goal[idx].get("implementation") == "policy":
            return Policy.create_subtree(action_client, goal[idx], robot_name=robot_name)

        if 'object' in goal[idx].keys():
            obj = goal[idx]['object']
        elif 'obj' in goal[idx].keys():
            obj = goal[idx]['obj']
        else:
            raise RuntimeError("MOVE: No place object")

        destination = goal[idx]['destination']

        if 'destination_offset' in goal[idx].keys():
            destination_offset = goal[idx]['destination_offset'] 
        else:
            destination_offset = [0,0,0,0,0,0]

        s_init3 = MoveJoint.MOVEJ(name="Init",\
                                  action_client=action_client,\
                                  action_goal=blackboard.init_config,
                                  robot_name=robot_name)

        # ----------------- Place ---------------------
        place = py_trees.composites.Sequence(name="Place", memory=True)
        pose_est2 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj,
                                                             'destination': destination,
                                                             'destination_offset': destination_offset},
                                              tf_buffer=kwargs['tf_buffer'],
                                              robot_name=robot_name)

        s_move21 = MovePose.MOVEP(name="Top",\
                                  action_client=action_client,\
                                  action_goal={'pose': "Plan"+idx+"/place_top_pose"},
                                  robot_name=robot_name)
        s_move22 = MovePose.MOVEP(name="Approach",\
                                  action_client=action_client,\
                                  action_goal={'pose': "Plan"+idx+"/place_pose"},
                                  robot_name=robot_name)
        s_move23 = Gripper.GOTO(name="Open",\
                                action_client=action_client,\
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force,
                                timeout=1,
                                robot_name=robot_name)
        s_move24 = MovePose.MOVEP(name="Top",\
                                  action_client=action_client,\
                                  action_goal={'pose': "Plan"+idx+"/place_top_pose"},
                                  robot_name=robot_name)
        
        place.add_children([pose_est2, s_move21, s_move22, s_move23, s_move24, s_init3])
        return place
