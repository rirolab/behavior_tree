"""Trigger one head-camera IKEA world-model snapshot from the behavior tree."""

from __future__ import annotations

import re
import time
import json

import py_trees
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


def world_object_estimate(snapshot, object_id):
    """Return a world-frame estimate only when it belongs to this object."""
    if not isinstance(snapshot, dict):
        return None
    obj = next(
        (item for item in snapshot.get("world", []) if item.get("id") == object_id),
        None,
    )
    estimate = None if obj is None else obj.get("estimated_transform")
    if (
        obj is None or obj.get("depth_status") != "estimated"
        or not isinstance(estimate, dict)
        or estimate.get("parent") != "world"
        or estimate.get("child") != f"g1_{object_id}_estimate"
    ):
        return None
    return estimate


class UpdateWorldModel(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_name, timeout):
        super().__init__(name=name)
        self.service_name = str(service_name)
        self.timeout = float(timeout)
        self._client = None
        self._future = None
        self._deadline = None
        self._retry_after = -float("inf")
        self._snapshot_blackboard = self.attach_blackboard_client(
            name=f"{name}_snapshot", namespace="g1"
        )
        self._snapshot_blackboard.register_key(
            key="snapshot_id", access=py_trees.common.Access.WRITE
        )

    def setup(self, node):
        self._client = node.create_client(Trigger, self.service_name)

    def initialise(self):
        self._future = None
        self._deadline = time.monotonic() + self.timeout
        self._retry_after = -float("inf")
        self._snapshot_blackboard.snapshot_id = ""

    def update(self):
        if self._client is None:
            self.feedback_message = "world-model snapshot client was not set up"
            return py_trees.common.Status.FAILURE
        if self._future is None:
            if time.monotonic() < self._retry_after:
                return py_trees.common.Status.RUNNING
            if self._client.service_is_ready():
                self._future = self._client.call_async(Trigger.Request())
                self.feedback_message = "capturing IKEA head-camera scene"
            elif time.monotonic() >= self._deadline:
                self.feedback_message = f"{self.service_name} unavailable"
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        if self._future.done():
            try:
                response = self._future.result()
            except Exception as exc:
                self.feedback_message = f"world-model snapshot failed: {exc}"
                return py_trees.common.Status.FAILURE
            self.feedback_message = response.message
            if response.success:
                match = re.search(r"(?:^|\s)snapshot_id=([A-Za-z0-9_]+)", response.message)
                if match:
                    self._snapshot_blackboard.snapshot_id = match.group(1)
                return py_trees.common.Status.SUCCESS
            if time.monotonic() >= self._deadline:
                return py_trees.common.Status.FAILURE
            self._future = None
            self._retry_after = time.monotonic() + 0.3
            return py_trees.common.Status.RUNNING
        if time.monotonic() >= self._deadline:
            self.feedback_message = "world-model snapshot timed out"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self._future is not None:
            self._future.cancel()


class EnsureWorldObject(py_trees.behaviour.Behaviour):
    """Retry head snapshots until any requested object has a fresh world TF."""

    def __init__(self, name, object_ids, service_name, timeout):
        super().__init__(name=name)
        self.object_ids = (
            (object_ids,) if isinstance(object_ids, str) else tuple(object_ids)
        )
        self.object_id = self.object_ids[0]
        self.service_name = service_name
        self.timeout = float(timeout)
        self._client = None
        self._future = None
        self._world_snapshot = None
        self._deadline = None
        self._retry_after = 0.0
        self._snapshot_wait_until = 0.0
        self._last_snapshot_error = ""
        self._snapshot_blackboard = self.attach_blackboard_client(
            name=f"{name}_snapshot", namespace="g1"
        )
        self._snapshot_blackboard.register_key(
            key="snapshot_id", access=py_trees.common.Access.WRITE
        )

    def setup(self, node):
        self._client = node.create_client(Trigger, self.service_name)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._subscription = node.create_subscription(
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
        self._future = None
        self._deadline = time.monotonic() + self.timeout
        self._retry_after = 0.0
        self._snapshot_wait_until = 0.0
        self._last_snapshot_error = ""

    def update(self):
        now = time.monotonic()
        if self._client is None:
            self.feedback_message = "world-model snapshot client was not set up"
            return py_trees.common.Status.FAILURE

        if self._future is not None and self._future.done():
            try:
                response = self._future.result()
            except Exception as exc:
                self.feedback_message = f"world-model snapshot failed: {exc}"
                self._last_snapshot_error = self.feedback_message
            else:
                self.feedback_message = response.message
                if response.success:
                    match = re.search(r"(?:^|\s)snapshot_id=([A-Za-z0-9_]+)", response.message)
                    if match:
                        self._snapshot_blackboard.snapshot_id = match.group(1)
                        self._snapshot_wait_until = now + 0.5
                else:
                    self._last_snapshot_error = response.message
            self._future = None

        try:
            snapshot_id = self._snapshot_blackboard.get("snapshot_id")
        except KeyError:
            snapshot_id = ""
        snapshot = self._world_snapshot
        if snapshot_id and snapshot is not None and (
            str(snapshot.get("diagnostics_dir", "")).rsplit("/", 1)[-1] == snapshot_id
        ):
            available = next(
                (object_id for object_id in self.object_ids
                 if world_object_estimate(snapshot, object_id) is not None),
                None,
            )
            if available is not None:
                self.feedback_message = f"{available} localized in snapshot {snapshot_id}"
                return py_trees.common.Status.SUCCESS
            self.feedback_message = (
                f"{', '.join(self.object_ids)} have no world TF in "
                f"snapshot {snapshot_id}; recapturing"
            )
            self._last_snapshot_error = self.feedback_message
        elif now < self._snapshot_wait_until:
            self.feedback_message = "waiting for the matching world-model snapshot"

        if now >= self._deadline:
            self.feedback_message = (
                f"{', '.join(self.object_ids)} world TF unavailable after {self.timeout:g}s "
                f"of perception retries; last result: {self._last_snapshot_error}"
            )
            return py_trees.common.Status.FAILURE
        if self._future is None and now >= max(self._retry_after, self._snapshot_wait_until):
            if self._client.service_is_ready():
                self._future = self._client.call_async(Trigger.Request())
                self._retry_after = now + 1.0
                self.feedback_message = f"recapturing head scene for {', '.join(self.object_ids)}"
            else:
                self.feedback_message = f"waiting for {self.service_name}"
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self._future is not None and not self._future.done():
            self._future.cancel()


class WorldObjectDepth(py_trees.behaviour.Behaviour):
    """BT selector guard for one candidate in the latest perception snapshot."""

    def __init__(self, name, object_id, *, has_depth):
        super().__init__(name=name)
        self.object_id = object_id
        self.has_depth = bool(has_depth)
        self._world_snapshot = None
        self._deadline = None
        self._snapshot_blackboard = self.attach_blackboard_client(
            name=f"{name}_snapshot", namespace="g1"
        )
        self._snapshot_blackboard.register_key(
            key="snapshot_id", access=py_trees.common.Access.READ
        )

    def setup(self, node):
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._subscription = node.create_subscription(
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
        self._deadline = time.monotonic() + 3.0

    def update(self):
        try:
            snapshot_id = self._snapshot_blackboard.get("snapshot_id")
        except KeyError:
            snapshot_id = ""
        snapshot = self._world_snapshot
        if not snapshot_id or snapshot is None or (
            str(snapshot.get("diagnostics_dir", "")).rsplit("/", 1)[-1] != snapshot_id
        ):
            self.feedback_message = "waiting for the matching world-model snapshot"
            return (
                py_trees.common.Status.RUNNING
                if time.monotonic() < self._deadline
                else py_trees.common.Status.FAILURE
            )
        available = world_object_estimate(snapshot, self.object_id) is not None
        self.feedback_message = (
            f"{self.object_id} {'has' if available else 'has no'} depth "
            f"in snapshot {snapshot_id}"
        )
        return (
            py_trees.common.Status.SUCCESS if available == self.has_depth
            else py_trees.common.Status.FAILURE
        )
