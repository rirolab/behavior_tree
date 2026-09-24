import json
from types import SimpleNamespace

import py_trees
import pytest
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

from behavior_tree.jobs.g1_jobs import (
    G1GripperJob, G1MoveToWorldObjectJob, G1PerceptionJob,
    G1StandCartesianJob, G1WaitJob,
)
from behavior_tree.subtrees import G1Cartesian, G1Perception, G1Wait
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


def test_ikea_perception_step_has_no_arm_assignment():
    job = G1PerceptionJob.__new__(G1PerceptionJob)
    step = {
        "primitive_action": "g1_perceive_ikea",
        "client": "perception",
        "service": "/g1/update_world_model",
        "timeout": 15.0,
    }
    assert job.validate_step(step) == StepValidationResult.ACCEPT_GOAL
    assert job.validate_step({**step, "robot": ["left_arm"]}) == StepValidationResult.REJECT_GOAL
    subtree = job.create_root(None, goal={"1": step})
    assert subtree.service_name == "/g1/update_world_model"


def test_ikea_perception_leaf_waits_for_snapshot_and_uses_service_result():
    class Future:
        def __init__(self):
            self.response = None

        def done(self):
            return self.response is not None

        def result(self):
            return self.response

    future = Future()
    client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda _request: future,
    )
    node = SimpleNamespace(create_client=lambda _type, _name: client)
    leaf = G1Perception.UpdateWorldModel("snapshot", "/g1/update_world_model", 5.0)
    leaf.setup(node)
    leaf.initialise()
    assert leaf.update() == py_trees.common.Status.RUNNING
    future.response = SimpleNamespace(success=False, message="table was not detected")
    assert leaf.update() == py_trees.common.Status.RUNNING
    leaf._deadline = 0.0
    leaf._retry_after = -float("inf")
    assert leaf.update() == py_trees.common.Status.RUNNING
    assert leaf.update() == py_trees.common.Status.FAILURE
    assert "table" in leaf.feedback_message
    leaf.initialise()
    assert leaf.update() == py_trees.common.Status.RUNNING
    future.response = SimpleNamespace(
        success=True, message="published world model; snapshot_id=scene_new"
    )
    assert leaf.update() == py_trees.common.Status.SUCCESS
    assert leaf._snapshot_blackboard.get("snapshot_id") == "scene_new"


def test_world_object_goal_uses_right_arm_and_world_offset():
    job = G1MoveToWorldObjectJob.__new__(G1MoveToWorldObjectJob)
    job._node = SimpleNamespace(locomotion_names=["g1"])
    step = {
        "primitive_action": "g1_move_to_world_object",
        "client": "cartesian",
        "locomotion": "g1",
        "robot": ["right_arm"],
        "object_id": "leg_1",
        "fallback_object_ids": ["leg_2"],
        "offset_world": [0.0, 0.0, 0.05],
        "timeout": 5.0,
    }
    assert job.validate_step(step) == StepValidationResult.ACCEPT_GOAL
    root = job.create_root(
        object(), goal={"1": step},
        locomotion_clients={"g1": object()},
        locomotion_status_topics={"g1": "/g1/locomotion_client/goal_status"},
    )
    assert root.children[1].object_ids == ("leg_1", "leg_2")
    fallback = root.children[2]
    assert [branch.name for branch in fallback.children] == [
        "Approach_leg_1", "Approach_leg_2"
    ]
    assert fallback.children[0].children[-1].object_id == "leg_1"
    assert fallback.children[1].children[-1].object_id == "leg_2"
    assert fallback.children[1].children[-1].robot_name == "right_arm"
    assert job.validate_step({**step, "offset_world": [0.0, 0.0, float("nan")]}) == StepValidationResult.REJECT_GOAL
    assert job.validate_step({**step, "fallback_object_ids": ["leg_1"]}) == StepValidationResult.REJECT_GOAL


def test_world_object_leaf_waits_for_matching_snapshot_and_tf():
    sent = []
    action_client = SimpleNamespace(call_async=lambda request: sent.append(json.loads(request.data)))
    transform = TransformStamped()
    transform.transform.translation.x = 1.2
    transform.transform.translation.y = 2.3
    transform.transform.translation.z = 0.4
    transform.transform.rotation.y = 2 ** -0.5
    transform.transform.rotation.w = 2 ** -0.5
    node = SimpleNamespace(
        tf_buffer=SimpleNamespace(lookup_transform=lambda *_args: transform),
        create_subscription=lambda *_args: None,
    )
    leaf = G1Cartesian.MoveToWorldObject(
        "MoveToLeg", action_client, "right_arm", "leg_1", [0.0, 0.0, 0.05], 5.0
    )
    leaf.setup(node)
    snapshot_blackboard = py_trees.blackboard.Client(name="test_snapshot", namespace="g1")
    snapshot_blackboard.register_key("snapshot_id", py_trees.common.Access.WRITE)
    snapshot_blackboard.snapshot_id = "scene_new"
    leaf.initialise()
    estimate = {
        "parent": "world",
        "child": "g1_leg_1_estimate",
        "translation_m": [1.2, 2.3, 0.4],
        "rotation_xyzw": [0.0, 2 ** -0.5, 0.0, 2 ** -0.5],
    }
    world = [{"id": "leg_1", "depth_status": "estimated", "estimated_transform": estimate}]
    leaf._world_callback(String(data=json.dumps({"diagnostics_dir": "outputs/scene_old", "world": world})))
    assert leaf.update() == py_trees.common.Status.RUNNING
    assert not sent
    leaf._world_callback(String(data=json.dumps({"diagnostics_dir": "outputs/scene_new", "world": world})))
    transform.transform.translation.x = 0.9
    assert leaf.update() == py_trees.common.Status.RUNNING
    assert not sent
    transform.transform.translation.x = 1.2
    assert leaf.update() == py_trees.common.Status.RUNNING
    assert sent[0]["robot_name"] == "right_arm"
    target = sent[0]["goal"]["target_pose"]
    assert target["frame_id"] == "world"
    assert target["position"] == [1.2, 2.3, 0.45]
    assert target["orientation"] == estimate["rotation_xyzw"]

    leaf.initialise()
    leaf._world_callback(String(data=json.dumps({
        "diagnostics_dir": "outputs/scene_new",
        "world": [{"id": "leg_1", "depth_status": "unavailable"}],
    })))
    assert leaf.update() == py_trees.common.Status.FAILURE
    leaf.terminate(py_trees.common.Status.FAILURE)
    assert "leg_1 has no world TF" in leaf.feedback_message
    assert len(sent) == 1


def test_world_object_retries_perception_until_target_has_a_fresh_tf():
    class Future:
        def __init__(self):
            self.response = None

        def done(self):
            return self.response is not None

        def result(self):
            return self.response

    futures = []
    client = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda _request: futures.append(Future()) or futures[-1],
    )
    node = SimpleNamespace(
        create_client=lambda _type, _name: client,
        create_subscription=lambda *_args: None,
    )
    leaf = G1Perception.EnsureWorldObject(
        "EnsureLeg", "leg_1", "/g1/update_world_model", 15.0
    )
    leaf.setup(node)
    blackboard = py_trees.blackboard.Client(name="test_retry_snapshot", namespace="g1")
    blackboard.register_key("snapshot_id", py_trees.common.Access.WRITE)
    blackboard.snapshot_id = "scene_without_depth"
    leaf._world_callback(String(data=json.dumps({
        "diagnostics_dir": "outputs/scene_without_depth",
        "world": [{"id": "leg_1", "depth_status": "unavailable"}],
    })))
    leaf.initialise()
    assert leaf.update() == py_trees.common.Status.RUNNING
    assert len(futures) == 1
    futures[0].response = SimpleNamespace(
        success=True, message="published; snapshot_id=scene_with_depth"
    )
    assert leaf.update() == py_trees.common.Status.RUNNING
    leaf._world_callback(String(data=json.dumps({
        "diagnostics_dir": "outputs/scene_with_depth",
        "world": [{
            "id": "leg_1", "depth_status": "estimated",
            "estimated_transform": {
                "parent": "world", "child": "g1_leg_1_estimate"
            },
        }],
    })))
    assert leaf.update() == py_trees.common.Status.SUCCESS
    assert blackboard.snapshot_id == "scene_with_depth"


def test_depth_fallback_approaches_leg_2_without_sending_leg_1_goal():
    sent = []
    captures = []
    action_client = SimpleNamespace(call_async=lambda request: sent.append(json.loads(request.data)))
    tf = TransformStamped()
    tf.transform.translation.x = 0.35
    tf.transform.translation.y = -0.1
    tf.transform.translation.z = 0.9
    tf.transform.rotation.w = 1.0
    node = SimpleNamespace(
        tf_buffer=SimpleNamespace(lookup_transform=lambda *_args: tf),
        create_subscription=lambda *_args: None,
        create_client=lambda *_args: SimpleNamespace(
            service_is_ready=lambda: True,
            call_async=lambda request: captures.append(request),
        ),
    )
    fallback = G1Cartesian.create_world_object_subtree(
        action_client=action_client,
        robot_name="right_arm",
        object_id="leg_1",
        fallback_object_ids=["leg_2"],
        offset_world=[0.0, 0.0, 0.05],
        locomotion_client=object(),
        locomotion_status_topic="/g1/locomotion_client/goal_status",
        timeout=5.0,
    ).children[2]
    snapshot = {
        "diagnostics_dir": "outputs/scene_fallback",
        "world": [
            {"id": "leg_1", "depth_status": "unavailable"},
            {"id": "leg_2", "depth_status": "estimated", "estimated_transform": {
                "parent": "world", "child": "g1_leg_2_estimate",
                "translation_m": [0.35, -0.1, 0.9],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            }},
        ],
    }
    for leaf in fallback.iterate():
        if isinstance(leaf, (G1Perception.WorldObjectDepth, G1Cartesian.MoveToWorldObject)):
            leaf.setup(node)
            leaf._world_callback(String(data=json.dumps(snapshot)))
    blackboard = py_trees.blackboard.Client(name="test_fallback_snapshot", namespace="g1")
    blackboard.register_key("snapshot_id", py_trees.common.Access.WRITE)
    blackboard.snapshot_id = "scene_fallback"
    ensure = G1Perception.EnsureWorldObject(
        "EnsureEitherLeg", ("leg_1", "leg_2"), "/g1/update_world_model", 15.0
    )
    ensure.setup(node)
    ensure._world_callback(String(data=json.dumps(snapshot)))
    ensure.initialise()
    assert ensure.update() == py_trees.common.Status.SUCCESS
    assert not captures
    fallback.tick_once()
    assert fallback.status == py_trees.common.Status.RUNNING
    assert fallback.children[0].status == py_trees.common.Status.FAILURE
    assert fallback.children[1].status == py_trees.common.Status.RUNNING
    assert len(sent) == 1
    assert sent[0]["goal"]["target_pose"]["position"] == pytest.approx([0.35, -0.1, 0.95])


def test_arm_command_failure_does_not_fallback_to_another_leg():
    sent = []
    action_client = SimpleNamespace(call_async=lambda request: sent.append(json.loads(request.data)))
    tf = TransformStamped()
    tf.transform.translation.x = 0.3
    tf.transform.translation.z = 0.9
    tf.transform.rotation.w = 1.0
    node = SimpleNamespace(
        tf_buffer=SimpleNamespace(lookup_transform=lambda *_args: tf),
        create_subscription=lambda *_args: None,
    )
    fallback = G1Cartesian.create_world_object_subtree(
        action_client=action_client, robot_name="right_arm",
        object_id="leg_1", fallback_object_ids=["leg_2"],
        offset_world=[0.0, 0.0, 0.05],
        locomotion_client=object(),
        locomotion_status_topic="/g1/locomotion_client/goal_status",
        timeout=5.0,
    ).children[2]
    snapshot = {
        "diagnostics_dir": "outputs/scene_primary",
        "world": [{
            "id": object_id, "depth_status": "estimated",
            "estimated_transform": {
                "parent": "world", "child": f"g1_{object_id}_estimate",
                "translation_m": [0.3, 0.0, 0.9],
                "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
            },
        } for object_id in ("leg_1", "leg_2")],
    }
    for leaf in fallback.iterate():
        if isinstance(leaf, (G1Perception.WorldObjectDepth, G1Cartesian.MoveToWorldObject)):
            leaf.setup(node)
            leaf._world_callback(String(data=json.dumps(snapshot)))
    blackboard = py_trees.blackboard.Client(name="test_no_motion_fallback", namespace="g1")
    blackboard.register_key("snapshot_id", py_trees.common.Access.WRITE)
    blackboard.snapshot_id = "scene_primary"
    fallback.tick_once()
    assert len(sent) == 1
    first_move = fallback.children[0].children[-1]
    first_move.command_response_status = lambda: py_trees.common.Status.FAILURE
    fallback.tick_once()
    assert fallback.status == py_trees.common.Status.FAILURE
    assert len(sent) == 1


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


def test_dual_dex1_goal_routes_to_gripper_status_channels():
    job = G1GripperJob.__new__(G1GripperJob)
    step = {
        "primitive_action": "g1_gripper",
        "client": "gripper",
        "robot": ["left_arm", "right_arm"],
        "target_q": 5.4,
        "timeout": 8.0,
    }
    assert job.validate_step(step) == StepValidationResult.ACCEPT_GOAL
    root = job.create_root(
        {"left_arm": object(), "right_arm": object()},
        goal={"1": step},
        robot_names=step["robot"],
    )
    assert len(root.children) == 2
    assert {child.goal_channel for child in root.children} == {"gripper"}
    assert {child.robot_name for child in root.children} == set(step["robot"])


def test_dex1_goal_rejects_out_of_range_target():
    job = G1GripperJob.__new__(G1GripperJob)
    step = {
        "primitive_action": "g1_gripper",
        "client": "gripper",
        "robot": ["left_arm", "right_arm"],
        "target_q": -5.3,
        "timeout": 8.0,
    }
    assert job.validate_step(step) == StepValidationResult.REJECT_GOAL
