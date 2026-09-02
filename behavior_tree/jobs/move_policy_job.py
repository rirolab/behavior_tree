import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.validation_utils import StepValidationResult
from behavior_tree.subtrees import MovePose, Gripper, Policy, WorldModel, MoveBlend


##############################################################################
# Behaviours
##############################################################################


class Move(base_job.BaseJob):
    """
    A move job variant that uses policy execution for the pick approach.
    """

    def __init__(self, node):
        """
        Tune into a channel for incoming goal requests.
        """
        super(Move, self).__init__(node)
        self.init_blackboard_parameters()

    def acceptable_step(self, step):
        """
        Check whether this job should accept a grounding step.
        """
        # Route only explicit policy-backed object moves through this job.
        if step.get("primitive_action") != "move_policy":
            return False

        # Check if the step has the number of robots required for this job.
        elif not self.check_robot_count(step, num_robot_required=1):
            return False

        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable step is well-formed enough to keep the goal.
        """
        # Check if the step has object/destination fields and optional policy skill ids.
        if self.acceptable_step(step):
            if ("object" not in step and "obj" not in step) or "destination" not in step:
                return StepValidationResult.REJECT_GOAL
            for key in ("top_policy_skill_id", "approach_policy_skill_id"):
                if key in step and not str(step[key]).strip():
                    return StepValidationResult.REJECT_GOAL
            # Validate optional blend timing before creating MoveBlend composites.
            if step.get("does_blend", False) or "blend_duration" in step:
                if "blend_duration" not in step:
                    return StepValidationResult.REJECT_GOAL
                try:
                    if float(step["blend_duration"]) <= 0.0:
                        return StepValidationResult.REJECT_GOAL
                except (TypeError, ValueError):
                    return StepValidationResult.REJECT_GOAL
            return StepValidationResult.ACCEPT_GOAL
        else:
            return StepValidationResult.NOT_APPLICABLE

    def incoming(self, msg):
        """
        Incoming goal callback.
        """
        # Store the full grounding goal only when this job owns at least one step.
        if self.goal:
            self._node.get_logger().error("move_policy_job: rejecting new goal, previous still in the pipeline")
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
        """
        # Check if the step is acceptable before building the subtree.
        if not self.acceptable_step(goal[idx]):
            return None

        # Create blackboard handles shared with the existing pick job.
        blackboard = py_trees.blackboard.Client(namespace=robot_name)
        blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)

        # Resolve object and policy skill knobs from the grounding step.
        step_goal = goal[idx]
        obj = step_goal["object"] if "object" in step_goal else step_goal["obj"]
        blend_duration = step_goal.get("blend_duration")
        enable_blend = bool(step_goal.get("does_blend", False) or blend_duration is not None)
        if enable_blend:
            blend_duration = float(blend_duration)
        top_policy_goal = {
            "primitive_action": "policy_execute",
            "skill_id": step_goal.get("top_policy_skill_id", "fr3_cube_rule_policy_jt"),
            "timeout": step_goal.get("top_policy_timeout", step_goal.get("timeout", 20.0)),
        }
        approach_policy_goal = {
            "primitive_action": "policy_execute",
            "skill_id": step_goal.get("approach_policy_skill_id", "fr3_cube_precise_rule_policy_jt"),
            "timeout": step_goal.get("approach_policy_timeout", step_goal.get("timeout", 20.0)),
        }

        # Build the pick plan, replacing only Top2 and Approach with policy execution.
        pose_est1 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict={'target': obj},
                                              tf_buffer=kwargs['tf_buffer'],
                                              robot_name=robot_name)
        s_move12 = Gripper.GOTO(name="Open",
                                action_client=action_client,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force,
                                timeout=1,
                                robot_name=robot_name)
        s_move10 = MovePose.MOVEPROOT(name="Top1",
                                      action_client=action_client,
                                      action_goal={'pose': "Plan"+idx+"/grasp_top_pose"},
                                      robot_name=robot_name,
                                      timeout=2)
        s_move11 = Policy.MOVEBYPOLICY(name="Top2",
                                       action_client=action_client,
                                       action_goal=top_policy_goal,
                                       timeout=float(top_policy_goal["timeout"]),
                                       robot_name=robot_name)
        s_move13 = Policy.MOVEBYPOLICY(name="Approach",
                                       action_client=action_client,
                                       action_goal=approach_policy_goal,
                                       timeout=float(approach_policy_goal["timeout"]),
                                       robot_name=robot_name)

        # Run only through s_move13 while collecting policy debug data.
        pick = py_trees.composites.Sequence(name="MovePolicyPick", memory=True)
        if enable_blend:
            # Blend only the trajectory-to-policy pick entry; policy-to-policy stays live.
            s_pick_approach_blend = MoveBlend.MoveBlend(
                name="PickApproachBlend",
                action_client=action_client,
                timeout=sum(float(child.timeout) for child in [s_move10, s_move11]),
                blend_duration=blend_duration,
                robot_name=robot_name,
                children=[s_move10, s_move11],
            )
            pick.add_children([pose_est1, s_move12, s_pick_approach_blend, s_move13])
        else:
            # Run non-blended pick motions in the original order.
            pick.add_children([pose_est1, s_move12, s_move10, s_move11, s_move13])

        # Keep the existing job root name while disabling actions after s_move13.
        task = py_trees.composites.Sequence(name="MovePolicy", memory=True)
        task.add_child(pick)
        return task
