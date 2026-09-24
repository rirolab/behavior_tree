"""Grounding jobs for G1 locomotion, waits, and Cartesian TCP actions."""

from __future__ import annotations

import json
import math

from behavior_tree.jobs import base_job
from behavior_tree.subtrees import G1Cartesian, G1Locomotion, G1Wait
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class G1BaseJob(base_job.BaseJob):
    """Common grounding ownership and task-id handling for G1 jobs."""

    primitive_action = ""

    def __init__(self, node):
        super().__init__(node)
        self.task_id = ""

    def _matches(self, step):
        return step.get("primitive_action") == self.primitive_action

    def acceptable_step(self, step):
        return self._matches(step)

    @staticmethod
    def _locomotion_name(step):
        return str(step.get("locomotion", "g1")).strip()

    def _has_locomotion_client(self, step):
        return self._locomotion_name(step) in getattr(
            self._node, "locomotion_names", []
        )

    def _locomotion_endpoint(self, step, clients, status_topics):
        name = self._locomotion_name(step)
        return clients.get(name), status_topics.get(name)

    def incoming(self, message):
        if self.goal:
            self._node.get_logger().error(
                f"{self.primitive_action}: previous goal is still pending"
            )
            return
        try:
            envelope = json.loads(message.data)
            grounding = envelope["params"]
        except (json.JSONDecodeError, KeyError, TypeError):
            return
        for index in range(len(grounding)):
            step = grounding.get(str(index + 1))
            if isinstance(step, dict) and self._matches(step):
                self.task_id = str(envelope.get("task_id", ""))
                self.goal = grounding
                return


class G1WalkJob(G1BaseJob):
    """Map a global G1 walking grounding step to the locomotion subtree."""

    primitive_action = "g1_walk"
    requires_robot_assignment = False

    def validate_step(self, step):
        if not self._matches(step):
            return StepValidationResult.NOT_APPLICABLE
        try:
            valid = (
                step.get("client") == "locomotion"
                and not make_string_list(step.get("robot"))
                and self._has_locomotion_client(step)
                and float(step.get("timeout", 120.0)) > 0.0
            )
        except (TypeError, ValueError):
            valid = False
        return (
            StepValidationResult.ACCEPT_GOAL
            if valid
            else StepValidationResult.REJECT_GOAL
        )

    def create_root(self, action_client, idx="1", goal=None, **kwargs):
        del action_client
        step = goal[idx]
        if self.validate_step(step) != StepValidationResult.ACCEPT_GOAL:
            return None
        client, status_topic = self._locomotion_endpoint(
            step,
            kwargs.get("locomotion_clients", {}),
            kwargs.get("locomotion_status_topics", {}),
        )
        if client is None or status_topic is None:
            return None
        return G1Locomotion.LocomotionCommand(
            name=f"G1Walk{idx}",
            action_type="startWalk",
            timeout=float(step.get("timeout", 120.0)),
            command_client=client,
            goal_status_topic=status_topic,
        )


class G1WaitJob(G1BaseJob):
    """Wait inside the BT sequence after the preceding step succeeds."""

    primitive_action = "g1_wait"
    requires_robot_assignment = False

    def validate_step(self, step):
        if not self._matches(step):
            return StepValidationResult.NOT_APPLICABLE
        try:
            duration = float(step.get("duration"))
            valid = (
                not make_string_list(step.get("robot"))
                and "client" not in step
                and math.isfinite(duration)
                and duration > 0.0
            )
        except (TypeError, ValueError):
            valid = False
        return (
            StepValidationResult.ACCEPT_GOAL
            if valid
            else StepValidationResult.REJECT_GOAL
        )

    def create_root(self, action_client, idx="1", goal=None, **kwargs):
        del action_client, kwargs
        step = goal[idx]
        if self.validate_step(step) != StepValidationResult.ACCEPT_GOAL:
            return None
        return G1Wait.WaitDuration(
            name=f"G1Wait{idx}",
            duration=float(step["duration"]),
        )


def _finite_vector(value, length):
    try:
        values = [float(item) for item in value]
    except (TypeError, ValueError):
        return False
    return len(values) == length and all(math.isfinite(item) for item in values)


def _valid_target_pose(value):
    if value == "current":
        return True
    if not isinstance(value, dict):
        return False
    if value.get("hold_current") is True:
        return True
    if not _finite_vector(value.get("position"), 3):
        return False
    has_rpy = "rpy" in value
    has_orientation = "orientation" in value
    if has_rpy == has_orientation:
        return False
    if has_rpy:
        return _finite_vector(value["rpy"], 3)
    if not _finite_vector(value["orientation"], 4):
        return False
    return any(abs(float(item)) > 1e-8 for item in value["orientation"])


class G1StandCartesianJob(G1BaseJob):
    """Stand, then send Cartesian targets to one or both G1 arm clients."""

    primitive_action = "g1_stand_cartesian"

    def validate_step(self, step):
        if not self._matches(step):
            return StepValidationResult.NOT_APPLICABLE
        try:
            robot_names = make_string_list(step.get("robot"))
            target_poses = step.get("target_poses")
            valid = (
                step.get("client") == "cartesian"
                and self._has_locomotion_client(step)
                and 1 <= len(robot_names) <= 2
                and len(set(robot_names)) == len(robot_names)
                and set(robot_names).issubset({"left_arm", "right_arm"})
                and isinstance(target_poses, dict)
                and set(target_poses) == set(robot_names)
                and all(_valid_target_pose(target_poses[name]) for name in robot_names)
                and float(step.get("timeout", 10.0)) > 0.0
                and math.isfinite(float(step.get("timeout", 10.0)))
            )
        except (TypeError, ValueError):
            valid = False
        return (
            StepValidationResult.ACCEPT_GOAL
            if valid
            else StepValidationResult.REJECT_GOAL
        )

    def create_root(
        self, action_client, idx="1", goal=None, robot_names=None, **kwargs
    ):
        step = goal[idx]
        if self.validate_step(step) != StepValidationResult.ACCEPT_GOAL:
            return None
        client, status_topic = self._locomotion_endpoint(
            step,
            kwargs.get("locomotion_clients", {}),
            kwargs.get("locomotion_status_topics", {}),
        )
        if client is None or status_topic is None:
            return None
        selected_robots = make_string_list(robot_names)
        if not selected_robots:
            selected_robots = make_string_list(kwargs.get("robot_name"))
        action_clients = (
            action_client
            if isinstance(action_client, dict)
            else {selected_robots[0]: action_client}
        )
        return G1Cartesian.create_subtree(
            action_clients=action_clients,
            target_poses=step["target_poses"],
            robot_names=selected_robots,
            locomotion_client=client,
            locomotion_status_topic=status_topic,
            timeout=float(step.get("timeout", 10.0)),
            name=f"G1StandCartesian{idx}",
        )
