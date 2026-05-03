import ast
import json
import time

import PyKDL
import py_trees
import rclpy
import std_msgs.msg as std_msgs
from tf2_ros import TransformException

from . import base_job
from behavior_tree.subtrees import DualArmMovePose, IsaacSceneCommand, MoveJoint


DUAL_ACTIONS = {
    "dual_init",
    "dual_move_joint",
    "dual_movej",
    "dual_joint",
    "dual_move_ring",
    "dual_ring_hover",
    "dual_ring_pose",
    "dual_ring_insert_sequence",
    "dual_ring_transfer",
}

DUAL_JOINT_ACTIONS = {
    "dual_init",
    "dual_move_joint",
    "dual_movej",
    "dual_joint",
}

DUAL_RING_ACTIONS = {
    "dual_move_ring",
    "dual_ring_hover",
    "dual_ring_pose",
}

DUAL_RING_SEQUENCE_ACTIONS = {
    "dual_ring_insert_sequence",
    "dual_ring_transfer",
}


def list_parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    value = node.get_parameter(name).value
    if isinstance(value, str):
        value = ast.literal_eval(value)
    return [float(v) for v in value]


def string_parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return str(node.get_parameter(name).value)


def string_list_parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    value = node.get_parameter(name).value
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (SyntaxError, ValueError):
            value = [value]
    return [str(v) for v in value]


def float_parameter(node, name, default):
    if not node.has_parameter(name):
        node.declare_parameter(name, default)
    return float(node.get_parameter(name).value)


def frame_from_transform(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return PyKDL.Frame(
        PyKDL.Rotation.Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        PyKDL.Vector(translation.x, translation.y, translation.z),
    )


def lookup_tf_frame(tf_buffer, node, target_frame, source_frame, transform_timeout):
    deadline = time.monotonic() + float(transform_timeout)
    last_error = None
    while rclpy.ok() and time.monotonic() < deadline:
        try:
            transform = tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
            return frame_from_transform(transform)
        except TransformException as exc:
            last_error = exc
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0.05)
            else:
                time.sleep(0.05)
    raise TransformException(
        f"transform {target_frame} <- {source_frame} unavailable: {last_error}"
    )


class RING_TARGET(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        tf_buffer,
        goal_key,
        world_frame,
        ring_frame,
        fallback_ring_frame,
        left_arm_base_frame,
        right_arm_base_frame,
        left_offset,
        right_offset,
        target_xyz_frame=None,
        target_xy_frame=None,
        target_z=None,
        target_z_delta=None,
        transform_timeout=2.0,
    ):
        super(RING_TARGET, self).__init__(name=name)
        self.tf_buffer = tf_buffer
        self.goal_key = goal_key
        self.world_frame = world_frame
        self.ring_frame = ring_frame
        self.fallback_ring_frame = fallback_ring_frame
        self.left_arm_base_frame = left_arm_base_frame
        self.right_arm_base_frame = right_arm_base_frame
        self.left_offset = [float(v) for v in left_offset]
        self.right_offset = [float(v) for v in right_offset]
        self.target_xyz_frame = target_xyz_frame
        self.target_xy_frame = target_xy_frame
        self.target_z = None if target_z is None else float(target_z)
        self.target_z_delta = None if target_z_delta is None else float(target_z_delta)
        self.transform_timeout = float(transform_timeout)
        self.node = None
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=self.goal_key, access=py_trees.common.Access.WRITE)

    def setup(self, node):
        self.node = node

    def update(self):
        try:
            self.blackboard.set(self.goal_key, self.make_dual_pose_goal())
        except Exception as exc:
            self.feedback_message = f"failed to compute ring target: {exc}"
            if self.node is not None:
                self.node.get_logger().error(self.feedback_message)
            return py_trees.common.Status.FAILURE
        self.feedback_message = "computed ring-relative dual pose goal"
        return py_trees.common.Status.SUCCESS

    def make_dual_pose_goal(self):
        ring_in_world, used_ring_frame = self.lookup_ring_frame()
        left_base_in_world = self.lookup_frame(self.world_frame, self.left_arm_base_frame)
        right_base_in_world = self.lookup_frame(self.world_frame, self.right_arm_base_frame)

        target_x = ring_in_world.p[0]
        target_y = ring_in_world.p[1]
        target_z = ring_in_world.p[2]
        target_source = used_ring_frame
        if self.target_xyz_frame:
            xyz_frame = self.lookup_frame(self.world_frame, self.target_xyz_frame)
            target_x = xyz_frame.p[0]
            target_y = xyz_frame.p[1]
            target_z = xyz_frame.p[2]
            target_source = f"{target_source}+xyz({self.target_xyz_frame})"
        if self.target_xy_frame:
            xy_frame = self.lookup_frame(self.world_frame, self.target_xy_frame)
            target_x = xy_frame.p[0]
            target_y = xy_frame.p[1]
            target_source = f"{target_source}+xy({self.target_xy_frame})"
        if self.target_z is not None:
            target_z = self.target_z
            target_source = f"{target_source}+z({self.target_z:.3f})"
        if self.target_z_delta is not None:
            target_z += self.target_z_delta
            target_source = f"{target_source}+dz({self.target_z_delta:.3f})"

        target_center = PyKDL.Vector(target_x, target_y, target_z)
        target_ring_in_world = PyKDL.Frame(ring_in_world.M, target_center)
        left_world = target_ring_in_world * PyKDL.Vector(*self.left_offset)
        right_world = target_ring_in_world * PyKDL.Vector(*self.right_offset)
        left_base = left_base_in_world.Inverse() * left_world
        right_base = right_base_in_world.Inverse() * right_world

        if self.node is not None:
            self.node.get_logger().info(
                "%s target from %s: left_world(%s)=[%.4f, %.4f, %.4f], "
                "right_world(%s)=[%.4f, %.4f, %.4f]"
                % (
                    self.name,
                    target_source,
                    self.world_frame,
                    left_world[0],
                    left_world[1],
                    left_world[2],
                    self.world_frame,
                    right_world[0],
                    right_world[1],
                    right_world[2],
                )
            )

        return {
            "source_frame": used_ring_frame,
            "world_frame": self.world_frame,
            "left": {
                "frame_id": self.left_arm_base_frame,
                "x": left_base[0],
                "y": left_base[1],
                "z": left_base[2],
            },
            "right": {
                "frame_id": self.right_arm_base_frame,
                "x": right_base[0],
                "y": right_base[1],
                "z": right_base[2],
            },
        }

    def lookup_ring_frame(self):
        try:
            return self.lookup_frame(self.world_frame, self.ring_frame), self.ring_frame
        except TransformException:
            if not self.fallback_ring_frame or self.fallback_ring_frame == self.ring_frame:
                raise
            return (
                self.lookup_frame(self.world_frame, self.fallback_ring_frame),
                self.fallback_ring_frame,
            )

    def lookup_frame(self, target_frame, source_frame):
        return lookup_tf_frame(
            self.tf_buffer,
            self.node,
            target_frame,
            source_frame,
            self.transform_timeout,
        )


class CURRENT_EE_Z_TARGET(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        tf_buffer,
        goal_key,
        world_frame,
        left_ee_frame,
        right_ee_frame,
        left_arm_base_frame,
        right_arm_base_frame,
        target_z=None,
        delta_z=None,
        transform_timeout=2.0,
    ):
        super(CURRENT_EE_Z_TARGET, self).__init__(name=name)
        self.tf_buffer = tf_buffer
        self.goal_key = goal_key
        self.world_frame = world_frame
        self.left_ee_frame = left_ee_frame
        self.right_ee_frame = right_ee_frame
        self.left_arm_base_frame = left_arm_base_frame
        self.right_arm_base_frame = right_arm_base_frame
        self.target_z = None if target_z is None else float(target_z)
        self.delta_z = None if delta_z is None else float(delta_z)
        self.transform_timeout = float(transform_timeout)
        self.node = None
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=self.goal_key, access=py_trees.common.Access.WRITE)

    def setup(self, node):
        self.node = node

    def update(self):
        try:
            self.blackboard.set(self.goal_key, self.make_dual_pose_goal())
        except Exception as exc:
            self.feedback_message = f"failed to compute current-ee z target: {exc}"
            if self.node is not None:
                self.node.get_logger().error(self.feedback_message)
            return py_trees.common.Status.FAILURE
        self.feedback_message = "computed current-ee z-only dual pose goal"
        return py_trees.common.Status.SUCCESS

    def make_dual_pose_goal(self):
        if self.target_z is None and self.delta_z is None:
            raise ValueError("CURRENT_EE_Z_TARGET requires target_z or delta_z")

        left_ee_in_world = self.lookup_frame(self.world_frame, self.left_ee_frame)
        right_ee_in_world = self.lookup_frame(self.world_frame, self.right_ee_frame)
        left_base_in_world = self.lookup_frame(self.world_frame, self.left_arm_base_frame)
        right_base_in_world = self.lookup_frame(self.world_frame, self.right_arm_base_frame)

        left_z = self.resolve_target_z(left_ee_in_world.p[2])
        right_z = self.resolve_target_z(right_ee_in_world.p[2])
        left_world = PyKDL.Vector(left_ee_in_world.p[0], left_ee_in_world.p[1], left_z)
        right_world = PyKDL.Vector(right_ee_in_world.p[0], right_ee_in_world.p[1], right_z)
        left_base = left_base_in_world.Inverse() * left_world
        right_base = right_base_in_world.Inverse() * right_world

        if self.node is not None:
            source = (
                f"current ee + dz({self.delta_z:.3f})"
                if self.delta_z is not None
                else f"current ee xy + z({self.target_z:.3f})"
            )
            self.node.get_logger().info(
                "dual current-ee target from %s: left_world=[%.4f, %.4f, %.4f], "
                "right_world=[%.4f, %.4f, %.4f]"
                % (
                    source,
                    left_world[0],
                    left_world[1],
                    left_world[2],
                    right_world[0],
                    right_world[1],
                    right_world[2],
                )
            )

        return {
            "source_frame": f"{self.left_ee_frame},{self.right_ee_frame}",
            "world_frame": self.world_frame,
            "left": {
                "frame_id": self.left_arm_base_frame,
                "x": left_base[0],
                "y": left_base[1],
                "z": left_base[2],
            },
            "right": {
                "frame_id": self.right_arm_base_frame,
                "x": right_base[0],
                "y": right_base[1],
                "z": right_base[2],
            },
        }

    def resolve_target_z(self, current_z):
        if self.delta_z is not None:
            return current_z + self.delta_z
        return self.target_z

    def lookup_frame(self, target_frame, source_frame):
        return lookup_tf_frame(
            self.tf_buffer,
            self.node,
            target_frame,
            source_frame,
            self.transform_timeout,
        )


class FINGERTIP_AVERAGE_Z_COMMAND(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        tf_buffer,
        command_key,
        world_frame,
        left_fingertip_frames,
        right_fingertip_frames,
        fingertip_z_offset=0.0,
        gravity_enabled=False,
        transform_timeout=2.0,
    ):
        super(FINGERTIP_AVERAGE_Z_COMMAND, self).__init__(name=name)
        self.tf_buffer = tf_buffer
        self.command_key = command_key
        self.world_frame = world_frame
        self.left_fingertip_frames = self.normalise_frames(left_fingertip_frames)
        self.right_fingertip_frames = self.normalise_frames(right_fingertip_frames)
        self.fingertip_z_offset = float(fingertip_z_offset)
        self.gravity_enabled = bool(gravity_enabled)
        self.transform_timeout = float(transform_timeout)
        self.node = None
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key=self.command_key,
            access=py_trees.common.Access.WRITE,
        )

    def setup(self, node):
        self.node = node

    def update(self):
        try:
            left_z = self.average_frame_z(self.left_fingertip_frames)
            right_z = self.average_frame_z(self.right_fingertip_frames)
            z = 0.5 * (left_z + right_z) + self.fingertip_z_offset
            self.blackboard.set(
                self.command_key,
                {
                    "action_type": "teleportActiveRing",
                    "z": z,
                    "gravity_enabled": self.gravity_enabled,
                },
            )
        except Exception as exc:
            self.feedback_message = f"failed to compute fingertip average z: {exc}"
            if self.node is not None:
                self.node.get_logger().error(self.feedback_message)
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"active ring target z={z:.4f}"
        if self.node is not None:
            self.node.get_logger().info(self.feedback_message)
        return py_trees.common.Status.SUCCESS

    def average_frame_z(self, frames):
        z_values = []
        for frame in frames:
            tip = lookup_tf_frame(
                self.tf_buffer,
                self.node,
                self.world_frame,
                frame,
                self.transform_timeout,
            )
            z_values.append(tip.p[2])
        return sum(z_values) / float(len(z_values))

    @staticmethod
    def normalise_frames(frames):
        if isinstance(frames, str):
            return [frames]
        return [str(frame) for frame in frames]


class Move(base_job.BaseJob):
    def __init__(self, node):
        super(Move, self).__init__(node)

        self.blackboard.register_key(key="left_init_config", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="right_init_config", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="left_arm_base_frame", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="right_arm_base_frame", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_frame", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_fallback_frame", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_world_frame", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_left_offset", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_right_offset", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_timeout", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="left_ee_frame", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="right_ee_frame", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="left_fingertip_frames", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="right_fingertip_frames", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="fingertip_z_offset", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_mold_frame", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_scene_command_service", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_scene_status_topic", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_hover_left_offset", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_hover_right_offset", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_grasp_left_offset", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_grasp_right_offset", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_release_left_offset", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_release_right_offset", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_lift_delta_z", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_ring_mold_delta_z", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_gripper_timeout", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_scene_timeout", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="dual_init_timeout", access=py_trees.common.Access.WRITE)
        self.blackboard.left_init_config = list_parameter(
            self._node,
            "left_init_config",
            [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0],
        )
        self.blackboard.right_init_config = list_parameter(
            self._node,
            "right_init_config",
            [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0],
        )
        self.blackboard.left_arm_base_frame = string_parameter(
            self._node,
            "left_arm_base_frame",
            "left_fr3_link0",
        )
        self.blackboard.right_arm_base_frame = string_parameter(
            self._node,
            "right_arm_base_frame",
            "right_fr3_link0",
        )
        self.blackboard.dual_ring_frame = string_parameter(
            self._node,
            "dual_ring_frame",
            "ring_00",
        )
        self.blackboard.dual_ring_fallback_frame = string_parameter(
            self._node,
            "dual_ring_fallback_frame",
            "active_ring",
        )
        self.blackboard.dual_world_frame = string_parameter(
            self._node,
            "world_frame",
            "world",
        )
        self.blackboard.dual_ring_left_offset = list_parameter(
            self._node,
            "dual_ring_left_offset",
            [0.0, 0.09348, 0.10]
        )
        self.blackboard.dual_ring_right_offset = list_parameter(
            self._node,
            "dual_ring_right_offset",
            [0.0, -0.09348, 0.10]
        )
        self.blackboard.dual_ring_timeout = float_parameter(
            self._node,
            "dual_ring_timeout",
            5.0,
        )
        self.blackboard.left_ee_frame = string_parameter(
            self._node,
            "left_ee_frame",
            "left_fr3_hand_tcp",
        )
        self.blackboard.right_ee_frame = string_parameter(
            self._node,
            "right_ee_frame",
            "right_fr3_hand_tcp",
        )
        self.blackboard.left_fingertip_frames = string_list_parameter(
            self._node,
            "left_fingertip_frames",
            ["left_fr3_hand_tcp"],
        )
        self.blackboard.right_fingertip_frames = string_list_parameter(
            self._node,
            "right_fingertip_frames",
            ["right_fr3_hand_tcp"],
        )
        self.blackboard.fingertip_z_offset = float_parameter(
            self._node,
            "fingertip_z_offset",
            -0.114,
        )
        self.blackboard.dual_mold_frame = string_parameter(
            self._node,
            "dual_mold_frame",
            "ring_mold",
        )
        self.blackboard.dual_scene_command_service = string_parameter(
            self._node,
            "dual_scene_command_service",
            "/scene/command",
        )
        self.blackboard.dual_scene_status_topic = string_parameter(
            self._node,
            "dual_scene_status_topic",
            "/scene/command_status",
        )
        self.blackboard.dual_ring_hover_left_offset = list_parameter(
            self._node,
            "dual_ring_hover_left_offset",
            self.blackboard.dual_ring_left_offset,
        )
        self.blackboard.dual_ring_hover_right_offset = list_parameter(
            self._node,
            "dual_ring_hover_right_offset",
            self.blackboard.dual_ring_right_offset,
        )
        self.blackboard.dual_ring_grasp_left_offset = list_parameter(
            self._node,
            "dual_ring_grasp_left_offset",
            [0.0, 0.09348, 0.0],
        )
        self.blackboard.dual_ring_grasp_right_offset = list_parameter(
            self._node,
            "dual_ring_grasp_right_offset",
            [0.0, -0.09348, 0.0],
        )
        self.blackboard.dual_ring_release_left_offset = list_parameter(
            self._node,
            "dual_ring_release_left_offset",
            [0.0, 0.13, 0.05],
        )
        self.blackboard.dual_ring_release_right_offset = list_parameter(
            self._node,
            "dual_ring_release_right_offset",
            [0.0, -0.13, 0.05],
        )
        self.blackboard.dual_ring_lift_delta_z = float_parameter(
            self._node,
            "dual_ring_lift_delta_z",
            0.20,
        )
        self.blackboard.dual_ring_mold_delta_z = float_parameter(
            self._node,
            "dual_ring_mold_delta_z",
            0.10,
        )
        self.blackboard.dual_gripper_timeout = float_parameter(
            self._node,
            "dual_gripper_timeout",
            1.0,
        )
        self.blackboard.dual_scene_timeout = float_parameter(
            self._node,
            "dual_scene_timeout",
            2.0,
        )
        self.blackboard.dual_init_timeout = float_parameter(
            self._node,
            "dual_init_timeout",
            5.0,
        )

    def incoming(self, msg):
        if self.goal:
            self._node.get_logger().error("dual_arm_job: rejecting new goal, previous still in the pipeline")
            return

        grounding = json.loads(msg.data)["params"]
        for i in range(len(grounding.keys())):
            action = grounding[str(i + 1)].get("primitive_action", "")
            if action in DUAL_ACTIONS:
                self.goal = grounding
                break

    def create_root(self, action_client, idx="1", goal=std_msgs.Empty(), **kwargs):
        command = dict(goal[idx])
        nested_goal = command.get("goal")
        if isinstance(nested_goal, str):
            nested_goal = json.loads(nested_goal)
        if isinstance(nested_goal, dict):
            merged = dict(nested_goal)
            merged.update(command)
            command = merged

        action = command.get("primitive_action", command.get("action", ""))
        if action not in DUAL_ACTIONS:
            return None

        blackboard = py_trees.blackboard.Client()
        blackboard.register_key(key="left_init_config", access=py_trees.common.Access.READ)
        blackboard.register_key(key="right_init_config", access=py_trees.common.Access.READ)
        blackboard.register_key(key="left_arm_base_frame", access=py_trees.common.Access.READ)
        blackboard.register_key(key="right_arm_base_frame", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_frame", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_fallback_frame", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_world_frame", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_left_offset", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_right_offset", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_timeout", access=py_trees.common.Access.READ)
        blackboard.register_key(key="left_ee_frame", access=py_trees.common.Access.READ)
        blackboard.register_key(key="right_ee_frame", access=py_trees.common.Access.READ)
        blackboard.register_key(key="left_fingertip_frames", access=py_trees.common.Access.READ)
        blackboard.register_key(key="right_fingertip_frames", access=py_trees.common.Access.READ)
        blackboard.register_key(key="fingertip_z_offset", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_mold_frame", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_scene_command_service", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_scene_status_topic", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_hover_left_offset", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_hover_right_offset", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_grasp_left_offset", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_grasp_right_offset", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_release_left_offset", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_release_right_offset", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_lift_delta_z", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_ring_mold_delta_z", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_gripper_timeout", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_scene_timeout", access=py_trees.common.Access.READ)
        blackboard.register_key(key="dual_init_timeout", access=py_trees.common.Access.READ)

        if action in DUAL_RING_SEQUENCE_ACTIONS:
            return self.create_ring_insert_sequence(
                action_client=action_client,
                idx=idx,
                command=command,
                tf_buffer=kwargs["tf_buffer"],
                blackboard=blackboard,
            )

        if action in DUAL_RING_ACTIONS:
            timeout = float(command.get("timeout", command.get("timeout_sec", blackboard.dual_ring_timeout)))
            left_offset = command.get("left_offset", blackboard.dual_ring_left_offset)
            right_offset = command.get("right_offset", blackboard.dual_ring_right_offset)
            goal_key = f"DualRingPlan{idx}/dual_pose_goal"
            root = py_trees.composites.Sequence(name="DualRingMove", memory=True)
            root.add_children(
                [
                    RING_TARGET(
                        name=f"DualRingPlan{idx}",
                        goal_key=goal_key,
                        tf_buffer=kwargs["tf_buffer"],
                        world_frame=command.get("world_frame", blackboard.dual_world_frame),
                        ring_frame=command.get("target_frame", command.get("object", blackboard.dual_ring_frame)),
                        fallback_ring_frame=command.get(
                            "fallback_frame",
                            blackboard.dual_ring_fallback_frame,
                        ),
                        left_arm_base_frame=command.get(
                            "left_arm_base_frame",
                            blackboard.left_arm_base_frame,
                        ),
                        right_arm_base_frame=command.get(
                            "right_arm_base_frame",
                            blackboard.right_arm_base_frame,
                        ),
                        left_offset=left_offset,
                        right_offset=right_offset,
                        transform_timeout=float(
                            command.get("transform_timeout", blackboard.dual_ring_timeout)
                        ),
                    ),
                    DualArmMovePose.DualArmMoveP(
                        name="DualMovePose",
                        action_client=action_client,
                        action_type="dualMovePose",
                        action_goal=goal_key,
                        timeout=timeout,
                    ),
                ]
            )
            return root

        if action == "dual_init":
            positions = list(blackboard.left_init_config) + list(blackboard.right_init_config)
        elif action in DUAL_JOINT_ACTIONS:
            positions = Move.extract_positions(command)
        else:
            return None

        timeout = float(command.get("timeout", command.get("timeout_sec", 3.0)))
        root = py_trees.composites.Sequence(name="DualArm", memory=True)
        root.add_child(
            MoveJoint.MOVEJ(
                name="DualMoveJoint",
                action_client=action_client,
                action_goal=positions,
                timeout=timeout,
            )
        )
        return root

    def create_ring_insert_sequence(self, action_client, idx, command, tf_buffer, blackboard):
        timeout = float(command.get("timeout", command.get("timeout_sec", blackboard.dual_ring_timeout)))
        straight_timeout = float(command.get("straight_timeout", timeout))
        gripper_timeout = float(command.get("gripper_timeout", blackboard.dual_gripper_timeout))
        scene_timeout = float(command.get("scene_timeout", blackboard.dual_scene_timeout))
        init_timeout = float(command.get("init_timeout", blackboard.dual_init_timeout))
        transform_timeout = float(command.get("transform_timeout", blackboard.dual_ring_timeout))
        straight_steps = command.get("straight_steps")

        world_frame = command.get("world_frame", blackboard.dual_world_frame)
        ring_frame = command.get("target_frame", command.get("object", blackboard.dual_ring_frame))
        fallback_ring_frame = command.get("fallback_frame", blackboard.dual_ring_fallback_frame)
        mold_frame = command.get("mold_frame", command.get("mold", blackboard.dual_mold_frame))
        left_base = command.get("left_arm_base_frame", blackboard.left_arm_base_frame)
        right_base = command.get("right_arm_base_frame", blackboard.right_arm_base_frame)
        left_fingertip_frames = command.get(
            "left_fingertip_frames",
            command.get("left_ee_frame", blackboard.left_fingertip_frames),
        )
        right_fingertip_frames = command.get(
            "right_fingertip_frames",
            command.get("right_ee_frame", blackboard.right_fingertip_frames),
        )
        fingertip_z_offset = float(
            command.get("fingertip_z_offset", blackboard.fingertip_z_offset)
        )
        scene_command_topic = command.get("scene_command_topic", blackboard.dual_scene_command_service)
        scene_status_topic = command.get("scene_status_topic", blackboard.dual_scene_status_topic)

        hover_left_offset = command.get(
            "hover_left_offset",
            blackboard.dual_ring_hover_left_offset,
        )
        hover_right_offset = command.get(
            "hover_right_offset",
            blackboard.dual_ring_hover_right_offset,
        )
        grasp_left_offset = command.get(
            "grasp_left_offset",
            blackboard.dual_ring_grasp_left_offset,
        )
        grasp_right_offset = command.get(
            "grasp_right_offset",
            blackboard.dual_ring_grasp_right_offset,
        )
        release_left_offset = command.get(
            "release_left_offset",
            blackboard.dual_ring_release_left_offset,
        )
        release_right_offset = command.get(
            "release_right_offset",
            blackboard.dual_ring_release_right_offset,
        )
        lift_delta_z = float(command.get("lift_delta_z", blackboard.dual_ring_lift_delta_z))
        if "mold_delta_z" in command:
            mold_delta_z = float(command["mold_delta_z"])
            mold_absolute_z = None
        elif "mold_absolute_z" in command:
            mold_delta_z = None
            mold_absolute_z = float(command["mold_absolute_z"])
        else:
            mold_delta_z = float(blackboard.dual_ring_mold_delta_z)
            mold_absolute_z = None

        prefix = f"DualRingInsert{idx}"
        hover_key = f"{prefix}/hover_goal"
        fingertip_command_key = f"{prefix}/fingertip_scene_command"
        lift_key = f"{prefix}/lift_goal"
        mold_key = f"{prefix}/mold_goal"
        release_key = f"{prefix}/release_goal"
        init_positions = list(blackboard.left_init_config) + list(blackboard.right_init_config)

        root = py_trees.composites.Sequence(name="DualRingInsertSequence", memory=True)
        root.add_children(
            [
                self.make_ring_target(
                    name=f"{prefix}/PlanHover",
                    goal_key=hover_key,
                    tf_buffer=tf_buffer,
                    world_frame=world_frame,
                    ring_frame=ring_frame,
                    fallback_ring_frame=fallback_ring_frame,
                    left_arm_base_frame=left_base,
                    right_arm_base_frame=right_base,
                    left_offset=hover_left_offset,
                    right_offset=hover_right_offset,
                    transform_timeout=transform_timeout,
                ),
                self.make_dual_arm_command(
                    name="DualMoveNearRingTop",
                    action_client=action_client,
                    action_type="dualMovePose",
                    action_goal=hover_key,
                    timeout=timeout,
                ),
                IsaacSceneCommand.SCENE_COMMAND(
                    name="DisableRingGravityBeforeTeleport",
                    command_topic=scene_command_topic,
                    status_topic=scene_status_topic,
                    command={
                        "action_type": "setActiveRingGravity",
                        "enabled": False,
                    },
                    timeout=scene_timeout,
                ),
                FINGERTIP_AVERAGE_Z_COMMAND(
                    name=f"{prefix}/FingerAverageZ",
                    tf_buffer=tf_buffer,
                    command_key=fingertip_command_key,
                    world_frame=world_frame,
                    left_fingertip_frames=left_fingertip_frames,
                    right_fingertip_frames=right_fingertip_frames,
                    fingertip_z_offset=fingertip_z_offset,
                    gravity_enabled=False,
                    transform_timeout=transform_timeout,
                ),
                IsaacSceneCommand.SCENE_COMMAND(
                    name="TeleportRingToFingerAverageZ",
                    command_topic=scene_command_topic,
                    status_topic=scene_status_topic,
                    command=fingertip_command_key,
                    timeout=scene_timeout,
                ),
                self.make_dual_arm_command(
                    name="DualGripperClose",
                    action_client=action_client,
                    action_type="dualGripperClose",
                    timeout=gripper_timeout,
                ),
                IsaacSceneCommand.SCENE_COMMAND(
                    name="EnableRingGravityAfterGrasp",
                    command_topic=scene_command_topic,
                    status_topic=scene_status_topic,
                    command={
                        "action_type": "setActiveRingGravity",
                        "enabled": True,
                    },
                    timeout=scene_timeout,
                ),
                CURRENT_EE_Z_TARGET(
                    name=f"{prefix}/PlanLift",
                    goal_key=lift_key,
                    tf_buffer=tf_buffer,
                    world_frame=world_frame,
                    left_ee_frame=command.get("left_ee_frame", blackboard.left_ee_frame),
                    right_ee_frame=command.get("right_ee_frame", blackboard.right_ee_frame),
                    left_arm_base_frame=left_base,
                    right_arm_base_frame=right_base,
                    delta_z=lift_delta_z,
                    transform_timeout=transform_timeout,
                ),
                self.make_dual_arm_command(
                    name="DualMoveLiftRingStraight",
                    action_client=action_client,
                    action_type="dualMovePoseStraight",
                    action_goal=lift_key,
                    timeout=straight_timeout,
                    extra=self.straight_extra(straight_steps),
                ),
                self.make_ring_target(
                    name=f"{prefix}/PlanMoldAlign",
                    goal_key=mold_key,
                    tf_buffer=tf_buffer,
                    world_frame=world_frame,
                    ring_frame=ring_frame,
                    fallback_ring_frame=fallback_ring_frame,
                    left_arm_base_frame=left_base,
                    right_arm_base_frame=right_base,
                    left_offset=grasp_left_offset,
                    right_offset=grasp_right_offset,
                    target_xyz_frame=mold_frame,
                    target_z=mold_absolute_z,
                    target_z_delta=mold_delta_z,
                    transform_timeout=transform_timeout,
                ),
                self.make_dual_arm_command(
                    name="DualMoveToMoldStraight",
                    action_client=action_client,
                    action_type="dualMovePoseStraight",
                    action_goal=mold_key,
                    timeout=straight_timeout,
                    extra=self.straight_extra(straight_steps),
                ),
                # SceneCommand.SCENE_COMMAND(
                #     name="DisableRingGravityBeforeRelease",
                #     command_topic=scene_command_topic,
                #     status_topic=scene_status_topic,
                #     command={
                #         "action_type": "setActiveRingGravity",
                #         "enabled": False,
                #     },
                #     timeout=scene_timeout,
                # ),
                self.make_dual_arm_command(
                    name="DualGripperOpen",
                    action_client=action_client,
                    action_type="dualGripperOpen",
                    timeout=gripper_timeout,
                ),
                self.make_ring_target(
                    name=f"{prefix}/PlanRelease",
                    goal_key=release_key,
                    tf_buffer=tf_buffer,
                    world_frame=world_frame,
                    ring_frame=ring_frame,
                    fallback_ring_frame=fallback_ring_frame,
                    left_arm_base_frame=left_base,
                    right_arm_base_frame=right_base,
                    left_offset=release_left_offset,
                    right_offset=release_right_offset,
                    target_xyz_frame=mold_frame,
                    target_z=mold_absolute_z,
                    target_z_delta=mold_delta_z,
                    transform_timeout=transform_timeout,
                ),
                self.make_dual_arm_command(
                    name="DualMoveRelease",
                    action_client=action_client,
                    action_type="dualMovePose",
                    action_goal=release_key,
                    timeout=timeout,
                ),
                # SceneCommand.SCENE_COMMAND(
                #     name="EnableRingGravityAfterRelease",
                #     command_topic=scene_command_topic,
                #     status_topic=scene_status_topic,
                #     command={
                #         "action_type": "setActiveRingGravity",
                #         "enabled": True,
                #     },
                #     timeout=scene_timeout,
                # ),
                self.make_dual_arm_command(
                    name="DualMoveInitialSeparate",
                    action_client=action_client,
                    action_type="dualMoveJointSeparate",
                    action_goal=init_positions,
                    timeout=init_timeout,
                ),
            ]
        )
        return root

    def make_ring_target(self, **kwargs):
        return RING_TARGET(**kwargs)

    def make_dual_arm_command(
        self,
        name,
        action_client,
        action_type,
        action_goal=None,
        timeout=3.0,
        extra=None,
    ):
        return DualArmMovePose.DualArmMoveP(
            name=name,
            action_client=action_client,
            action_type=action_type,
            action_goal=action_goal,
            timeout=timeout,
            extra=extra,
        )

    @staticmethod
    def straight_extra(straight_steps):
        if straight_steps is None:
            return None
        return {"steps": int(straight_steps)}

    @staticmethod
    def extract_positions(command):
        positions = command.get("positions")
        if positions is not None:
            if len(positions) != 14:
                raise ValueError("positions must contain 14 values for dual-arm commands")
            return [float(value) for value in positions]

        left = Move.first_present(
            command,
            ("left", "left_positions", "left_joint_positions", "left_config", "arm_l"),
        )
        right = Move.first_present(
            command,
            ("right", "right_positions", "right_joint_positions", "right_config", "arm_r"),
        )
        if left is None or right is None:
            raise KeyError("dual command requires left/right joint positions or a 14-value positions list")
        if len(left) != 7 or len(right) != 7:
            raise ValueError("left and right positions must each contain 7 values")
        return [float(value) for value in left] + [float(value) for value in right]

    @staticmethod
    def first_present(command, keys):
        for key in keys:
            if key in command:
                return command[key]
        return None
