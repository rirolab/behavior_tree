import copy, sys
import threading
import json
import rclpy

import py_trees, py_trees_ros
import py_trees.console as console

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

        # beahviors
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

        if goal[idx].get("implementation") == "policy":
            # Real-deploy: wrap the learned pick policy with joint-space waypoints
            # (pre_init -> init -> policy -> return) instead of pose-estimated
            # MoveP approaches, which are unreliable before a learned policy on
            # the real robot.
            init_config = blackboard.init_config
            pre_init_config = self._read_optional_config(robot_name, "pre_init_config")
            if pre_init_config is None:
                pre_init_config = init_config
            s_pre_init = MoveJoint.MOVEJ(
                name="PreInit",
                action_client=action_client,
                action_goal=pre_init_config,
                robot_name=robot_name,
            )
            s_init = MoveJoint.MOVEJ(
                name="Init",
                action_client=action_client,
                action_goal=init_config,
                robot_name=robot_name,
            )
            policy_pick = Policy.MOVEBYPOLICY(
                name="MoveByPolicy",
                action_client=action_client,
                action_goal=goal[idx],
                timeout=float(goal[idx].get("timeout", 20.0)),
                robot_name=robot_name,
            )
            s_return = MoveJoint.MOVEJ(
                name="Return",
                action_client=action_client,
                action_goal=init_config,
                robot_name=robot_name,
            )
            root.add_children([s_pre_init, s_init, policy_pick, s_return])
            return root
        
        # ------------ Compute -------------------------
        s_init1 = MoveJoint.MOVEJ(name="Init",\
                                  action_client=action_client,\
                                  action_goal=blackboard.init_config,
                                  robot_name=robot_name)
        s_init2 = MoveJoint.MOVEJ(name="Init2",\
                                  action_client=action_client,\
                                  action_goal=blackboard.init_config,
                                  robot_name=robot_name)

        # ----------------- Pick ---------------------
        pose_est1 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj},
                                              tf_buffer=kwargs['tf_buffer'],
                                              robot_name=robot_name)
        s_move11 = MovePose.MOVEP(name="Top",\
                                  action_client=action_client,\
                                  action_goal={'pose': "Plan"+idx+"/grasp_top_pose"},
                                  robot_name=robot_name)
        s_move12 = Gripper.GOTO(name="Open",
                                action_client=action_client,\
                                action_goal=blackboard.gripper_open_pos,\
                                force=blackboard.gripper_open_force,\
                                timeout=1,
                                robot_name=robot_name)
        s_move13 = MovePose.MOVEP(name="Approach",\
                                  action_client=action_client,\
                                  action_goal={'pose': "Plan"+idx+"/grasp_pose"},
                                  robot_name=robot_name)
        s_move14 = Gripper.GOTO(name="Close",\
                                action_client=action_client,\
                                action_goal=blackboard.gripper_close_pos,\
                                force=blackboard.gripper_close_force,\
                                timeout=5,
                                robot_name=robot_name)
        s_move15 = MovePose.MOVEP(name="Top",\
                                  action_client=action_client,\
                                  action_goal={'pose': "Plan"+idx+"/grasp_top_pose"},
                                  robot_name=robot_name)

        pick = py_trees.composites.Sequence(name="Pick", memory=True)
        pick.add_children([pose_est1, s_init1, s_move11, s_move12, s_move13, s_move14, s_move15, s_init2])
        return pick

    
