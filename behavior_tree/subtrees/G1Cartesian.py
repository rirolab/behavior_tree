"""G1 Cartesian command leaves and stand-then-move composition."""

from __future__ import annotations

import json
import math
import time

from action_msgs.msg import GoalStatus
import numpy as np
import py_trees
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from riro_srvs.srv import StringGoalStatus
from std_msgs.msg import String
from tf2_ros import TransformException

from behavior_tree.subtrees import G1Locomotion, G1Perception, Move, MoveParallel


STAND_TRANSITION_DEADLINE_SECONDS = 30.0
OBJECT_TF_WAIT_SECONDS = 3.0


class MoveCartesian(Move.MOVE):
    """Send one TCP pose to a G1 Cartesian complex action client."""

    def update(self):
        if self.cmd_req is None:
            self.feedback_message = "Cartesian action client is unavailable"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16, dtype=np.uint8)
            request = StringGoalStatus.Request()
            request.data = json.dumps(
                self._make_command(
                    "moveCartesian",
                    self.action_goal,
                    uuid=self.goal_uuid_des.tolist(),
                    enable_wait=False,
                )
            )
            self.future = self.cmd_req.call_async(request)
            self.sent_goal = True
            self.feedback_message = "Sending a Cartesian TCP goal"
            return py_trees.common.Status.RUNNING

        response_status = self.command_response_status()
        if response_status is not None:
            return response_status
        if self.current_goal_id() is None or not self.goal_matches_blackboard():
            return py_trees.common.Status.RUNNING

        status = self.current_goal_status()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            return py_trees.common.Status.SUCCESS
        if status in (
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_CANCELED,
        ):
            self.feedback_message = "FAILURE"
            return py_trees.common.Status.FAILURE
        self.feedback_message = "running"
        return py_trees.common.Status.RUNNING


class MoveToWorldObject(MoveCartesian):
    """Resolve an object TF from the just-completed perception snapshot."""

    def __init__(self, name, action_client, robot_name, object_id,
                 offset_world, timeout):
        super().__init__(
            name=name,
            action_client=action_client,
            action_goal=None,
            timeout=timeout,
            robot_name=robot_name,
        )
        self.object_id = object_id
        self.offset_world = tuple(offset_world)
        self._world_snapshot = None
        self._tf_buffer = None
        self._resolve_deadline = None
        self._snapshot_blackboard = self.attach_blackboard_client(
            name=f"{name}_snapshot", namespace="g1"
        )
        self._snapshot_blackboard.register_key(
            key="snapshot_id", access=py_trees.common.Access.READ
        )

    def setup(self, node):
        super().setup(node)
        self._tf_buffer = node.tf_buffer
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._world_subscription = node.create_subscription(
            String, "/world_model", self._world_callback, qos
        )

    def _world_callback(self, message):
        try:
            snapshot = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(snapshot, dict) and isinstance(snapshot.get("world"), list):
            self._world_snapshot = snapshot

    def initialise(self):
        super().initialise()
        self.action_goal = None
        self.goal_uuid_des = None
        self.future = None
        self._resolve_deadline = time.monotonic() + OBJECT_TF_WAIT_SECONDS

    def terminate(self, new_status):
        # A missing object TF fails before any goal exists. Preserve the
        # specific reason for the BT task status in that case.
        if self.action_goal is None:
            return
        super().terminate(new_status)

    def _wait_for_target(self, message):
        self.feedback_message = message
        if time.monotonic() >= self._resolve_deadline:
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def update(self):
        if self.action_goal is None:
            try:
                snapshot_id = self._snapshot_blackboard.get("snapshot_id")
            except KeyError:
                snapshot_id = ""
            if not snapshot_id:
                return self._wait_for_target("perception snapshot ID unavailable")
            snapshot = self._world_snapshot
            if snapshot is None or str(snapshot.get("diagnostics_dir", "")).rsplit("/", 1)[-1] != snapshot_id:
                return self._wait_for_target("waiting for the matching world-model snapshot")
            estimate = G1Perception.world_object_estimate(snapshot, self.object_id)
            child_frame = f"g1_{self.object_id}_estimate"
            if estimate is None:
                self.feedback_message = (
                    f"{self.object_id} has no world TF in the new perception snapshot"
                )
                return py_trees.common.Status.FAILURE
            try:
                transform = self._tf_buffer.lookup_transform(
                    "world", child_frame, Time()
                )
            except TransformException:
                return self._wait_for_target(f"waiting for world -> {child_frame} TF")
            position = transform.transform.translation
            orientation = transform.transform.rotation
            tf_xyz = [position.x, position.y, position.z]
            tf_xyzw = [orientation.x, orientation.y, orientation.z, orientation.w]
            model_xyz = estimate["translation_m"]
            model_xyzw = estimate["rotation_xyzw"]
            if (
                math.dist(tf_xyz, model_xyz) > 0.005
                or abs(abs(sum(a * b for a, b in zip(tf_xyzw, model_xyzw))) - 1.0) > 0.001
            ):
                return self._wait_for_target(
                    f"waiting for {child_frame} TF from the new snapshot"
                )
            self.action_goal = {
                "target_pose": {
                    "frame_id": "world",
                    "position": [
                        tf_xyz[axis] + self.offset_world[axis]
                        for axis in range(3)
                    ],
                    "orientation": tf_xyzw,
                }
            }
        return super().update()


def create_subtree(
    action_clients,
    target_poses,
    robot_names,
    locomotion_client,
    locomotion_status_topic,
    timeout,
    name="G1StandCartesian",
):
    """Request standing mode, then command the selected TCP poses in parallel."""
    timeout = float(timeout)
    hold_stand = G1Locomotion.LocomotionCommand(
        name="G1HoldStand",
        action_type="holdStand",
        timeout=STAND_TRANSITION_DEADLINE_SECONDS,
        command_client=locomotion_client,
        goal_status_topic=locomotion_status_topic,
    )
    arm_commands = [
        MoveCartesian(
            name=f"Cartesian_{robot_name}",
            action_client=action_clients[robot_name],
            action_goal={"target_pose": target_poses[robot_name]},
            timeout=timeout,
            robot_name=robot_name,
        )
        for robot_name in robot_names
    ]
    move_arms = MoveParallel.MoveParallel(
        name="G1MoveTCPs", children=arm_commands
    )
    root = py_trees.composites.Sequence(name=name, memory=True)
    root.add_children([hold_stand, move_arms])
    return root


def create_world_object_subtree(
    action_client, robot_name, object_id, offset_world,
    locomotion_client, locomotion_status_topic, timeout,
    perception_timeout=15.0, snapshot_service="/g1/update_world_model",
    fallback_object_ids=(),
    name="G1MoveToWorldObject",
):
    """Hold standing mode, then approach the first depth-backed candidate."""
    object_ids = (object_id, *fallback_object_ids)
    hold_stand = G1Locomotion.LocomotionCommand(
        name="G1HoldStandForObject",
        action_type="holdStand",
        timeout=STAND_TRANSITION_DEADLINE_SECONDS,
        command_client=locomotion_client,
        goal_status_topic=locomotion_status_topic,
    )
    ensure_object = G1Perception.EnsureWorldObject(
        name=f"Ensure_{object_id}_fallback_world_TF",
        object_ids=object_ids,
        service_name=snapshot_service,
        timeout=perception_timeout,
    )
    fallback = py_trees.composites.Selector(
        name="G1ObjectDepthFallback", memory=True
    )
    for index, candidate in enumerate(object_ids):
        attempt = py_trees.composites.Sequence(
            name=f"Approach_{candidate}", memory=True
        )
        # Fallback is permitted only because earlier candidates lack depth.
        # An arm-action failure must never redirect the hand to another leg.
        guards = [
            G1Perception.WorldObjectDepth(
                name=f"{earlier}_no_depth_before_{candidate}",
                object_id=earlier,
                has_depth=False,
            )
            for earlier in object_ids[:index]
        ]
        guards.append(G1Perception.WorldObjectDepth(
            name=f"{candidate}_depth_available",
            object_id=candidate,
            has_depth=True,
        ))
        move = MoveToWorldObject(
            name=f"CartesianTo_{candidate}_{robot_name}",
            action_client=action_client,
            robot_name=robot_name,
            object_id=candidate,
            offset_world=offset_world,
            timeout=timeout,
        )
        attempt.add_children([*guards, move])
        fallback.add_child(attempt)
    root = py_trees.composites.Sequence(name=name, memory=True)
    root.add_children([hold_stand, ensure_object, fallback])
    return root
