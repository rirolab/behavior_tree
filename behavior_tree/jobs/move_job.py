import copy, sys
import threading
import rclpy
import json

import py_trees, py_trees_ros
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.skill_registry_utils import skill_init_config
from behavior_tree.utils.validation_utils import StepValidationResult
from behavior_tree.subtrees import MoveJoint, MovePose, Gripper, Policy, WorldModel
from behavior_tree.subtrees.MoveBlend import (
    OVERLAP_DISPATCH_ON_START_PARAM,
    OVERLAP_THRESHOLD_PARAM,
    MoveBlend,
)


def _move_sequence(name, children):
    """A sequence whose consecutive arm motions overlap.

    Falls back to strict sequential wherever a child exposes no progress (a
    gripper op, a world-model query), and everywhere when the threshold is 1.0.
    """
    seq = MoveBlend(name=name, threshold_param=OVERLAP_THRESHOLD_PARAM,
                          dispatch_param=OVERLAP_DISPATCH_ON_START_PARAM,
                          progress_threshold=1.0)
    seq.add_children(children)
    return seq


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
        # Route object moves and direct joint moves through this job.
        if step.get("primitive_action") not in ["move", "move_joint"]:
            return False
        
        # Check if the step has the number of robots required for this job
        elif not self.check_robot_count(step, num_robot_required=1):
            return False

        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable step is well-formed enough to keep
        the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        # Check if the step has the required parameters for move
        if self.acceptable_step(step):
            # Case: Direct joint movement
            if step.get("primitive_action") == "move_joint":
                if "joint_positions" in step:
                    return StepValidationResult.ACCEPT_GOAL
                else:
                    return StepValidationResult.REJECT_GOAL

            # Case: Policy movement
            elif step.get("implementation") == "policy":
                if bool(step.get("skill_id")):
                    return StepValidationResult.ACCEPT_GOAL
                else:
                    return StepValidationResult.REJECT_GOAL

            # Case: Primitive movement
            elif ("object" in step) and ("destination" in step):
                return StepValidationResult.ACCEPT_GOAL

            # Case: Reject if the step is acceptable but mal-formed
            else:
                return StepValidationResult.REJECT_GOAL

            # Case: Not acceptable step
        else:
            return StepValidationResult.NOT_APPLICABLE

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            self._node.get_logger().error("MOVE: rejecting new goal, previous still in the pipeline")
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

        # Create a direct joint-move subtree from the grounding payload.
        if goal[idx].get("primitive_action") == "move_joint":
            return MoveJoint.MOVEJ(name="MoveJoint", action_client=action_client,
                                   action_goal=goal[idx]["joint_positions"],
                                   timeout=goal[idx].get("timeout", 3.0),
                                   robot_name=robot_name)

        # beahviors
        root = py_trees.composites.Sequence(name="Move", memory=True)
        blackboard = py_trees.blackboard.Client(namespace=robot_name)
        blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)

        if goal[idx].get("implementation") == "policy":
            # Real-deploy: wrap the learned policy with joint-space waypoints.
            # Sandwich: pre_init -> init -> policy -> init -> place -> open -> init -> pre_init -> home
            # The policy deposits the object, then the arm places/retreats home.
            init_config = blackboard.init_config
            pre_init_config = self._read_optional_config(robot_name, "pre_init_config")
            if pre_init_config is None:
                pre_init_config = init_config
            place_config = self._read_optional_config(robot_name, "place_config")
            home_config = self._read_optional_config(robot_name, "home_config")
            if place_config is None or home_config is None:
                raise RuntimeError(
                    "move_job policy branch requires 'place_config' and "
                    "'home_config' parameters"
                )
            s_pre_init_1 = MoveJoint.MOVEJ(name="PreInit", action_client=action_client,
                                           action_goal=pre_init_config, robot_name=robot_name)

            # Send the approach to the SKILL's trained start pose, not the arm's
            # generic init_config, when the registry knows one.
            #
            # Otherwise the arm is moved twice: here to the arm's init_config,
            # and then again by the executor's blocking 4-second pre-move to the
            # skill's. The second of those is dead time by construction -- it
            # runs inside the policy goal, so by the time it finishes the arm has
            # stopped and there is nothing left for the mixer to blend the policy
            # into. Approaching the right pose here is what makes the executor's
            # `init_pose_policy: expect` correct, and that pair is what removes
            # the 4 s.
            #
            # Falls back to init_config whenever the registry cannot answer (cac
            # not installed, unknown skill, a dual-arm skill), so this is inert
            # rather than wrong when it does not apply.
            approach_config = skill_init_config(
                goal[idx].get("skill_id"), arm_dof=len(init_config)
            )
            if approach_config is None:
                approach_config = init_config
            s_init_1 = MoveJoint.MOVEJ(name="Init", action_client=action_client,
                                       action_goal=approach_config,
                                       robot_name=robot_name)
            policy = Policy.create_subtree(action_client, goal[idx], robot_name=robot_name)
            s_init_2 = MoveJoint.MOVEJ(name="Init2", action_client=action_client,
                                       action_goal=init_config, robot_name=robot_name)
            s_place = MoveJoint.MOVEJ(name="Place", action_client=action_client,
                                      action_goal=place_config, robot_name=robot_name)
            s_open = Gripper.GOTO_VIA_ARM(name="Open", action_client=action_client,
                                          action_goal=blackboard.gripper_open_pos,
                                          timeout=2.0, robot_name=robot_name)
            s_init_3 = MoveJoint.MOVEJ(name="Init3", action_client=action_client,
                                       action_goal=init_config, robot_name=robot_name)
            s_pre_init_2 = MoveJoint.MOVEJ(name="PreInit2", action_client=action_client,
                                           action_goal=pre_init_config, robot_name=robot_name)
            s_home = MoveJoint.MOVEJ(name="Home", action_client=action_client,
                                     action_goal=home_config, robot_name=robot_name)
            # The seam this whole effort is about is inside this sequence --
            # s_init_1 -> policy -> s_init_2 -- so it has to be the overlapping
            # kind. The gripper op in the middle is not overlappable and stays a
            # strict barrier on its own, which is the behaviour that must not
            # change: opening before the arm arrives drops the object.
            wrapped = _move_sequence("MovePolicy", [
                s_pre_init_1, s_init_1, policy,
                s_init_2, s_place, s_open, s_init_3, s_pre_init_2, s_home,
            ])
            return wrapped

        obj         = goal[idx]['object']
        destination = goal[idx]['destination']
        
        # ----------------- Move Task ----------------        
        s_init3 = MoveJoint.MOVEJ(name="Init", action_client=action_client,
                                  action_goal=blackboard.init_config,
                                  robot_name=robot_name)

        # ----------------- Pick ---------------------
        pose_est1 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj},
                                              tf_buffer=kwargs['tf_buffer'],
                                              robot_name=robot_name)
        s_move10 = MovePose.MOVEPROOT(name="Top1",
                                      action_client=action_client,
                                      action_goal={'pose': "Plan"+idx+"/grasp_top_pose"},
                                      robot_name=robot_name)
        s_move11 = MovePose.MOVEP(name="Top2",
                                  action_client=action_client,
                                  action_goal={'pose': "Plan"+idx+"/grasp_top_pose"},
                                  robot_name=robot_name)
        s_move12 = Gripper.GOTO(name="Open",
                                action_client=action_client,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force,
                                timeout=1,
                                robot_name=robot_name)
        s_move13 = MovePose.MOVEP(name="Approach",
                                  action_client=action_client,
                                  action_goal={'pose': "Plan"+idx+"/grasp_pose"},
                                  robot_name=robot_name)
        s_move14 = Gripper.GOTO(name="Close",
                                action_client=action_client,
                                action_goal=blackboard.gripper_close_pos,
                                force=blackboard.gripper_close_force,
                                timeout=5,
                                robot_name=robot_name)
        s_move15 = MovePose.MOVEP(name="Top",
                                  action_client=action_client,
                                  action_goal={'pose': "Plan"+idx+"/grasp_top_pose"},
                                  robot_name=robot_name)

        # Open the gripper up front, not mid-chain: it is empty during the pick
        # approach and usually already open (the robot starts open and a place
        # leaves it open), so opening here is a harmless no-op that would only
        # force a stop if left between Top2 and Approach. Up front it lets
        # Top1->Top2->Approach blend as one descent; Close still breaks the chain
        # so the grasp pose is reached exactly before the gripper closes.
        pick = _move_sequence("MovePick",
            [pose_est1, s_move12, s_move10, s_move11, s_move13, s_move14, s_move15])


        # ----------------- Place ---------------------
        pose_est2 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj,
                                                             'destination': destination},
                                              tf_buffer=kwargs['tf_buffer'],
                                              robot_name=robot_name)
        s_move20 = MovePose.MOVEPROOT(name="Top1",
                                      action_client=action_client,
                                      action_goal={'pose': "Plan"+idx+"/place_top_pose"},
                                      robot_name=robot_name)
        s_move21 = MovePose.MOVEP(name="Top2", action_client=action_client,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"},
                                 robot_name=robot_name)
        s_move22 = MovePose.MOVEP(name="Approach", action_client=action_client,
                                 action_goal={'pose': "Plan"+idx+"/place_pose"},
                                 robot_name=robot_name)
        s_move23 = Gripper.GOTO(name="Open", action_client=action_client,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force,
                                timeout=1,
                                robot_name=robot_name)
        s_move24 = MovePose.MOVEP(name="Top", action_client=action_client,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"},
                                 robot_name=robot_name)
        
        # Overlap runs within the place chain: Top1->Top2->Approach blends, and
        # Top->Init blends; the gripper Open breaks the chain at the release.
        place = _move_sequence("MovePlace",
            [pose_est2, s_move20, s_move21, s_move22, s_move23, s_move24, s_init3])
        
        task = py_trees.composites.Sequence(name="Move", memory=True)
        task.add_children([pick, place])
        return task
