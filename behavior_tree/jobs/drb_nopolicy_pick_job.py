import copy, sys
import threading
import json
import rclpy

import py_trees, py_trees_ros
import py_trees.console as console

import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.validation_utils import StepValidationResult
from behavior_tree.subtrees import (
    IsaacSceneCommand,
    MoveJoint,
    MovePose,
    Gripper,
    Policy,
    WorldModel,
    RingWorldModel,
)


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
        # Check if the primitive action is pick
        if step.get("primitive_action") != "pick":
            return False

        # Check if the step has the number of robots required for this job
        elif not self.check_robot_count(step, num_robot_required=1):
            return False

        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable pick step is well-formed enough to keep
        the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        # Check if the step has the required parameters for pick
        if self.acceptable_step(step):
            if step.get("implementation") == "policy":
                if bool(step.get("skill_id")):
                    return StepValidationResult.ACCEPT_GOAL
                else:
                    return StepValidationResult.REJECT_GOAL
            elif ("object" in step) or ("obj" in step):
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
            self._node.get_logger().error("pick_job: rejecting new goal, previous still in the pipeline")
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

        # Time parameters
        GRIPPER_TIME = 0.25
        MOVE_TIME = 0.25

        # behaviors
        root = py_trees.composites.Sequence(name="Pick", memory=True)
        blackboard = py_trees.blackboard.Client(namespace=robot_name)
        blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)

        if 'object' in goal[idx].keys():
            obj = goal[idx]['object']
        elif 'obj' in goal[idx].keys():
            obj = goal[idx]['obj']
        else:
            console.logerror("Pick: No pick object")
            sys.exit()

        pose_est1 = WorldModel.POSE_ESTIMATOR(
            name="Plan" + idx,
            object_dict={'target': obj},
            tf_buffer=kwargs['tf_buffer'],
            robot_name=robot_name,
        )
        s_init1 = MovePose.MOVEP(
            name="PickInit",
            action_client=action_client,
            action_goal={'pose': "Plan" + idx + "/grasp_top_pose"},
            robot_name=robot_name,
            timeout=MOVE_TIME,
        )
        s_move1 = Gripper.GOTO(name="Open",
                                action_client=action_client,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force,
                                timeout=GRIPPER_TIME,
                                robot_name=robot_name)

        s_scene_cmd1 = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="TeleportActiveRingRigidToPickPose",
            command={
                "action_type": "teleportActiveRingRigid",
                "target_frame": "ring_stack_anchor_grasp_top",
                "offset_xyz": [0.0, 0.0, 0.0],
                "local_axis": "x",
                "angle_deg": 90.0,
            },
            timeout=5.0,
        )
        s_move2 = Gripper.GOTO(name="Close",
                                action_client=action_client,
                                action_goal=blackboard.gripper_close_pos,
                                force=blackboard.gripper_close_force,
                                timeout=GRIPPER_TIME,
                                robot_name=robot_name)
        s_scene_cmd2 = IsaacSceneCommand.ISAAC_SCENE_COMMAND(
            name="EnableActiveRingGravityAndDeformable",
            command={
                "action_type": "enableActiveRingGravityAndDeformable",
            },
            timeout=5.0,
        )
        root.add_children([pose_est1, s_init1, s_move1, s_scene_cmd1, s_move2, s_scene_cmd2])
        return root

    
