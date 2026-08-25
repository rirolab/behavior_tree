from enum import Enum, auto
from behavior_tree.utils.parameter_utils import make_string_list


class StepValidationResult(Enum):
    """
    Result of validating whether one step should reject the whole goal.
    """

    NOT_APPLICABLE = auto()
    ACCEPT_GOAL = auto()
    REJECT_GOAL = auto()


def validate_robot_names(robot_names, parameter_names):
    """
    Validate multi-robot names against available parameter namespaces.
    """
    # Require at least one robot name.
    if not robot_names:
        raise RuntimeError(
            "Invalid multi_dynamic_behavior_tree robot parameter: "
            "parameter [robot] must define at least one robot name"
        )

    # Require robot names to match parameter namespaces.
    parameter_namespaces = {
        parameter_name.split(".", 1)[0]
        for parameter_name in parameter_names
        if "." in parameter_name
    }
    robot_names_set = set(robot_names)
    if robot_names_set != parameter_namespaces:
        raise RuntimeError(
            "Invalid multi_dynamic_behavior_tree robot parameter: "
            f"parameter [robot] names {sorted(robot_names_set)} "
            f"must match parameter namespaces {sorted(parameter_namespaces)}"
        )

    # Reject blank robot names.
    blank_robot_names = [
        robot_name for robot_name in robot_names if robot_name.strip() == ""
    ]
    if blank_robot_names:
        raise RuntimeError(
            "Invalid multi_dynamic_behavior_tree robot parameter: "
            "parameter [robot] contains an empty robot name"
        )

    # Reject duplicate robot names.
    if len(set(robot_names)) != len(robot_names):
        raise RuntimeError(
            "Invalid multi_dynamic_behavior_tree robot parameter: "
            f"parameter [robot] contains duplicate robot names: {robot_names}"
        )


def validate_goal(goal, jobs, robot_names):
    """
    Validate that every grounding step can be routed by this tree instance.
    """
    # Check every step against optional robot routing and job ownership.
    robot_names = robot_names or []
    available_robot_names = set(robot_names)
    for idx in range(len(goal)):
        step_idx = str(idx + 1)
        step = goal.get(step_idx)
        if step is None:
            return False, f"{step_idx}: validate_goal rejected goal due to missing step"

        # Resolve omitted robot field only when the tree owns one named robot.
        grounding_robot_names = make_string_list(step.get("robot"))
        if not grounding_robot_names and len(robot_names) == 1:
            grounding_robot_names = [robot_names[0]]

        # Reject missing or unavailable robot assignments only for named robot trees.
        if robot_names:
            if (
                not grounding_robot_names
                or not set(grounding_robot_names).issubset(available_robot_names)
            ):
                return False, f"{step_idx}: validate_goal rejected goal due to invalid assignment"

        # Reject the goal if any job marks this step malformed.
        job_validation_result = []
        rejecting_jobs = []
        for job in jobs:
            result = job.validate_step(step)
            job_validation_result.append(result)
            if result == StepValidationResult.REJECT_GOAL:
                rejecting_jobs.append(job.__class__.__module__.split(".")[-1])
        if rejecting_jobs:
            return (
                False,
                f"{step_idx}: validate_goal rejected goal due to job validation failure "
                f"from {', '.join(rejecting_jobs)}",
            )

        # Reject ambiguous or unhandled steps.
        accept_count = sum(
            result == StepValidationResult.ACCEPT_GOAL
            for result in job_validation_result
        )
        if accept_count != 1:
            return (
                False,
                f"{step_idx}: validate_goal rejected goal because the step is accepted by "
                f"{accept_count} jobs, but should be accepted by exactly one job",
            )

    return True, ""

def collect_blend_validation(job_root):
    """
    Validate blend metadata attached to a generated job subtree.
    """
    # Collect every invalid blend marker inside the generated subtree.
    blend_reject_reasons = [
        f"{node.name}: {node.blend_reject_reason}"
        for node in job_root.iterate()
        if getattr(node, "blend_reject_reason", None)
    ]
    if blend_reject_reasons:
        return False, "; ".join(blend_reject_reasons)
    return True, ""
