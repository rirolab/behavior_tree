import math

import py_trees

from . import policy_job
from behavior_tree.subtrees import Policy
from behavior_tree.utils.validation_utils import StepValidationResult


# Define the six benchmark instructions for the supported Ideal/case1 sequence.
CASE1_INSTRUCTIONS = (
    "Pick up cream cheese from coffee table",
    "Place down cream cheese into white storage box placed at bottom side",
    "Pick up popcorn from coffee table",
    "Place down popcorn into white storage box placed at right side",
    "Pick up butter from coffee table",
    "Place down butter into white storage box placed at left side",
)

# Accept either registered model for the same six-step BT sequence.
SUPPORTED_SKILLS = {"robocerebra_openvla_case1", "robocerebra_smolvla_case1"}


class Move(policy_job.Move):
    """Run six instructed policy steps for RoboCerebra Ideal/case1."""

    def acceptable_step(self, step):
        # Route the dedicated sequence through the existing policy job lifecycle.
        return (
            step.get("primitive_action") == "robocerebra_policy_sequence"
            and self.check_robot_count(step, num_robot_required=1)
        )

    def validate_step(self, step):
        # Require the supported case and a sequence identifier for progress tracking.
        result = super().validate_step(step)
        if result == StepValidationResult.ACCEPT_GOAL:
            if (step.get("task_type"), step.get("case_name")) != ("Ideal", "case1"):
                return StepValidationResult.REJECT_GOAL
            if not step.get("sequence_id"):
                return StepValidationResult.REJECT_GOAL
            # Validate the policy selection and per-step deadline before building children.
            if step["skill_id"] not in SUPPORTED_SKILLS:
                return StepValidationResult.REJECT_GOAL
            try:
                timeout = float(step["timeout"])
            except (KeyError, TypeError, ValueError):
                return StepValidationResult.REJECT_GOAL
            if not math.isfinite(timeout) or timeout <= 0.0:
                return StepValidationResult.REJECT_GOAL
        return result

    def create_root(self, action_client, idx="1", goal=None, robot_name=None, **kwargs):
        # Assign each child its benchmark instruction and completion step.
        step = goal[idx]
        if not self.acceptable_step(step):
            return None
        root = py_trees.composites.Sequence(name="RoboCerebraIdealCase1", memory=True)
        for task_step, instruction in enumerate(CASE1_INSTRUCTIONS):
            policy_goal = dict(
                step, primitive_action="policy_execute", task_step=task_step,
                instruction=instruction,
            )
            root.add_child(Policy.MOVEBYPOLICY(
                name=f"{step['skill_id']}_{task_step + 1}",
                action_client=action_client,
                action_goal=policy_goal,
                timeout=float(step["timeout"]),
                robot_name=robot_name,
            ))
        return root
