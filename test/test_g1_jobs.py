from types import SimpleNamespace

import py_trees

from behavior_tree.jobs.g1_jobs import G1StandCartesianJob, G1WaitJob
from behavior_tree.subtrees import G1Wait
from behavior_tree.utils.validation_utils import StepValidationResult


def _job():
    job = G1StandCartesianJob.__new__(G1StandCartesianJob)
    job._node = SimpleNamespace(locomotion_names=["g1"])
    return job


def _wait_job():
    job = G1WaitJob.__new__(G1WaitJob)
    job._node = SimpleNamespace(locomotion_names=["g1"])
    return job


def test_wait_goal_without_robot_assignment_is_accepted():
    step = {
        "primitive_action": "g1_wait",
        "duration": 10.0,
    }
    assert _wait_job().validate_step(step) == StepValidationResult.ACCEPT_GOAL


def test_wait_goal_rejects_nonpositive_or_routed_duration():
    assert _wait_job().validate_step(
        {"primitive_action": "g1_wait", "duration": 0.0}
    ) == StepValidationResult.REJECT_GOAL


def test_wait_leaf_succeeds_only_after_its_duration(monkeypatch):
    clock = {"now": 100.0}
    monkeypatch.setattr(G1Wait.time, "monotonic", lambda: clock["now"])
    wait = G1Wait.WaitDuration(name="Wait", duration=10.0)

    wait.initialise()
    assert wait.update() == py_trees.common.Status.RUNNING
    clock["now"] = 109.999
    assert wait.update() == py_trees.common.Status.RUNNING
    clock["now"] = 110.0
    assert wait.update() == py_trees.common.Status.SUCCESS
    assert _wait_job().validate_step(
        {
            "primitive_action": "g1_wait",
            "duration": 10.0,
            "robot": ["left_arm"],
        }
    ) == StepValidationResult.REJECT_GOAL


def test_single_arm_cartesian_goal_is_accepted():
    step = {
        "primitive_action": "g1_stand_cartesian",
        "client": "cartesian",
        "locomotion": "g1",
        "robot": ["left_arm"],
        "target_poses": {
            "left_arm": {
                "frame_id": "pelvis",
                "position": [0.3, 0.2, 0.1],
                "rpy": [0.0, 0.0, 0.0],
            }
        },
        "timeout": 5.0,
    }
    assert _job().validate_step(step) == StepValidationResult.ACCEPT_GOAL


def test_joint_or_mismatched_cartesian_goal_is_rejected():
    step = {
        "primitive_action": "g1_stand_cartesian",
        "client": "cartesian",
        "locomotion": "g1",
        "robot": ["left_arm", "right_arm"],
        "target_poses": {"left_arm": "current"},
        "joint_positions": [0.0] * 14,
        "timeout": 5.0,
    }
    assert _job().validate_step(step) == StepValidationResult.REJECT_GOAL
