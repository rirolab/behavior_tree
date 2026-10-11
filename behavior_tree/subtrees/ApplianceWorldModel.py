"""Collect appliance state and publish Cartesian targets to the BT blackboard."""

# Reuse KDL frames and the repository's motion/status/cancellation behaviours.
import math
import time

import numpy as np
import PyKDL as kdl
import py_trees
import yaml
from mujoco_ros2_control_msgs.msg import FreeJointStateArray
from rclpy.time import Time
from sensor_msgs.msg import JointState

from trajectory_manager.geometry_conversions import (
    KDLframe2Pose, pose2KDLframe, transform2KDLframe,
)


class SceneState:
    def __init__(self, node, tf_buffer):
        # Load scene geometry once and share feedback across this job's tasks.
        self.node, self.tf_buffer = node, tf_buffer
        with open(node.get_parameter("appliance_config_file").value) as stream:
            self.config = yaml.safe_load(stream)
        self.joints, self.objects = {}, {}
        self.joint_time = self.object_time = 0.0
        self.subscriptions = [
            node.create_subscription(JointState, "/joint_states", self.joint_state, 10),
            node.create_subscription(FreeJointStateArray, "/mujoco_object_states", self.object_state, 10),
        ]

    def joint_state(self, msg):
        # Reject invalid feedback and retain measured passive hinge angles.
        if len(msg.name) != len(msg.position) or not np.isfinite(msg.position).all():
            self.node.get_logger().error("Invalid appliance joint feedback")
            return
        self.joints.update(zip(msg.name, msg.position))
        self.joint_time = time.monotonic()

    def object_state(self, msg):
        # MuJoCo free-body poses describe cube centres in simulator world.
        if msg.header.frame_id not in ("", "world"):
            return
        objects = {}
        for state in msg.free_joints:
            pose = state.pose.pose
            q = pose.orientation
            values = [pose.position.x, pose.position.y, pose.position.z, q.x, q.y, q.z, q.w]
            if state.pose.header.frame_id not in ("", "world") or not np.isfinite(values).all():
                return
            if abs(np.linalg.norm(values[3:]) - 1.0) > 0.01:
                return
            objects[state.name] = pose2KDLframe(pose)
        self.objects = objects
        self.object_time = time.monotonic()

    def ready(self):
        # Bound feedback age even when simulation time stops.
        return (time.monotonic() - min(self.joint_time, self.object_time) < 1.0
                and {"fridge_door_hinge", "microwave_door_hinge"} <= self.joints.keys()
                and {"red_cube", "blue_cube"} <= self.objects.keys())

    def frame(self, name):
        # Read the measured robot or cabinet frame from robot_state_publisher.
        transform = self.tf_buffer.lookup_transform("world", name, Time())
        return transform2KDLframe(transform.transform)

    def safe(self):
        # Stop active CAC motion when simulator feedback becomes stale.
        if not self.ready():
            self.node.get_logger().error("Appliance feedback stopped during motion")
            return False
        return True


class APPLIANCE_POSE_ESTIMATOR(py_trees.behaviour.Behaviour):
    def __init__(self, name, scene, step, output_key):
        # scene supplies feedback; step selects the task; output_key names its targets.
        super().__init__(name)
        self.scene, self.step, self.output_key = scene, step, output_key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(output_key, access=py_trees.common.Access.WRITE)

    def initialise(self):
        # Bound startup waiting without blocking subscriptions or tree cancellation.
        self.started = time.monotonic()

    def update(self):
        # Wait up to 30 wall-clock seconds for fresh door and cube feedback.
        # This subtree calculates targets; CAC and trajectory_manager execute motion.
        if not self.scene.ready():
            return (py_trees.common.Status.RUNNING if time.monotonic() - self.started < 30.0
                    else py_trees.common.Status.FAILURE)
        try:
            # Read the current cabinet and object state before calculating task targets.
            config, step = self.scene.config, self.step
            # base transforms cabinet-local coordinates into simulator world coordinates.
            base = self.scene.frame("appliance_stack_base")
            # Object frames use cube centres, not cube bottom surfaces.
            objects = self.scene.objects
            # Use the angle magnitude here; spec["sign"] supplies the opening direction.
            angle = abs(self.scene.joints[step["appliance"] + "_door_hinge"])
            # All output frames are world-frame targets for nc_right_grasp.
            # Positions are metres; rotation angles are radians unless converted below.
            name = step["appliance"]
            spec = config[name]
            # upright is the configured grasp rotation; front includes the cabinet rotation.
            upright = kdl.Rotation(*config["upright_rotation"])
            front = base.M * upright
            action = step["primitive_action"]
            targets = {}

            if action == "appliance_place":
                # pick is the world-frame side-grasp orientation, independent of cabinet yaw.
                pick = kdl.Rotation.RotZ(-math.pi / 2) * upright
                position = objects[step["object"]].p
                # transit is a fixed staging pose for this scene, not a collision-planned pose.
                targets["cube_pick_staging"] = kdl.Frame(pick, kdl.Vector(0.4, -0.24, 0.85 if name == "fridge" else 0.9))
                # approach stops 10 cm before the cube along world X; grasp reaches its centre.
                targets["cube_pick_approach"] = kdl.Frame(pick, position - kdl.Vector(0.1, 0, 0))
                targets["cube_pick_grasp"] = kdl.Frame(pick, position)
                # lift changes world Z while holding the cube's current world X and Y.
                targets["cube_pick_lift"] = kdl.Frame(pick, kdl.Vector(position[0], position[1], 0.9 if name == "fridge" else 0.85))
                # transfer and turn are fixed transit poses; turn adopts the cabinet-facing rotation.
                if name == "fridge":
                    targets["cube_place_transfer"] = kdl.Frame(pick, kdl.Vector(0.35, position[1], 0.9))
                    targets["cube_place_reorient"] = kdl.Frame(front, kdl.Vector(0.35, -0.05, 1.0))
                else:
                    targets["cube_place_reorient"] = kdl.Frame(front, kdl.Vector(0.4, -0.2, 0.85))
                # place converts the configured cabinet-local destination to world coordinates.
                targets["cube_place_target"] = kdl.Frame(front, base * kdl.Vector(*spec["place"]))
                # outside offsets 20 cm along the grasp frame's +Z axis, not world +Z.
                targets["cube_place_entry"] = kdl.Frame(front, targets["cube_place_target"].p + 0.20 * front.UnitZ())

                # Retract through the same entry pose after releasing the cube.
                targets["cube_place_retreat"] = targets["cube_place_entry"]

            else:
                # Sample the measured hinge arc with PyKDL; do not solve IK inside the BT.
                if action not in ("appliance_open", "appliance_close"):
                    raise ValueError(f"Unknown appliance action: {action}")
                opening = action == "appliance_open"
                # Both closing tasks push the panel; only opening tasks grasp a handle.
                panel = not opening
                # Name targets by task and contact type for readable BT keys.
                door_prefix = "door_open" if opening else "door_close"
                contact_prefix = door_prefix + ("_panel" if panel else "_handle")
                # Open to the configured angle or close to zero, with at most 2 degrees per segment.
                end = math.radians(spec["open_degrees"]) if opening else 0.0
                count = max(1, math.ceil(abs(end - angle) / math.radians(2)))
                frames = []
                for value in np.linspace(angle, end, count + 1):
                    turn = kdl.Rotation.RotZ(spec["sign"] * float(value))
                    # handle/panel is a hinge-relative offset; pivot is cabinet-local.
                    point = spec["panel"] if panel else spec["handle"]
                    # Rotate the contact point around the hinge, then transform it into world.
                    position = base * (kdl.Vector(*spec["pivot"]) + turn * kdl.Vector(*point))
                    # Start with the cabinet-facing orientation.
                    rotation = front
                    # The fridge grasp follows the door rotation with a fixed -55 degree yaw offset.
                    if name == "fridge" and opening:
                        rotation = base.M * turn * kdl.Rotation.RotZ(math.radians(-55)) * upright
                    # Partially follow the door to keep the wrist away from the protruding handle.
                    if panel:
                        fraction = 0.25
                        rotation = base.M * kdl.Rotation.RotZ(spec["sign"] * float(value) * fraction) * upright
                        # Turn the fingers sideways to clear the protruding handle.
                        rotation = rotation * kdl.Rotation.RotZ(math.pi / 2)
                    # Closed fingertips extend 17.9 mm beyond the grasp frame; request light tip contact.
                    if panel:
                        position += 0.017 * rotation.UnitZ()
                    frames.append(kdl.Frame(rotation, position))

                # transit is a scene-specific staging pose; grasp is the current contact pose.
                targets[f"{door_prefix}_staging"] = kdl.Frame(frames[0].M if opening else front, kdl.Vector(
                    0.35 if panel and name == "microwave" else 0.4, -0.2, 0.95 if panel and name == "microwave" else (1.045 if name == "fridge" else 0.72)))
                # Opening needs a grasp target; closing only needs a surface-contact path.
                if opening:
                    targets[f"{contact_prefix}_grasp"] = frames[0]
                # normal is the approach direction in world, derived from the grasp +Z axis.
                normal = front.UnitZ() if opening else frames[0].M.UnitZ()
                # Start the panel push just outside the measured door surface.
                clearance = 0.10 if opening else 0.06
                # Keep the contact orientation during approach.
                targets[f"{contact_prefix}_approach"] = kdl.Frame(frames[0].M if opening or panel else front,
                                                                frames[0].p + clearance * normal)
                if panel:
                    # Reach a raised clearance pose before moving the tips toward the panel.
                    targets["door_close_panel_above"] = kdl.Frame(frames[0].M, targets[f"{contact_prefix}_approach"].p + kdl.Vector(0, 0, 0.04))
                    # Give the microwave's joint-space approach more clearance from its open door.
                    if name == "microwave":
                        targets["door_close_panel_above"].p = frames[0].p + 0.20 * normal + kdl.Vector(0, 0, 0.10)
                    frames.insert(0, targets[f"{contact_prefix}_approach"])
                # Slow the complete contact arc for physical tracking, independent of waypoint count.
                targets[f"{door_prefix}_arc"] = frames
                targets[f"{door_prefix}_arc_seconds"] = 16.0
                # After closure, retreat 12 cm along the cabinet-facing grasp +Z axis.
                targets[f"{door_prefix}_retreat"] = kdl.Frame(frames[-1].M, frames[-1].p + 0.12 * front.UnitZ())
                if opening:
                    # After opening, retreat along the fridge grasp axis or raise 22 cm for microwave.
                    targets[f"{door_prefix}_retreat"] = (kdl.Frame(front, frames[-1].p + 0.12 * frames[-1].M.UnitZ())
                                          if name == "fridge" else
                                          kdl.Frame(frames[-1].M, frames[-1].p + kdl.Vector(0, 0, 0.22)))
                # home is a task staging pose, not the robot's spawn or joint-home configuration.
                targets[f"{door_prefix}_return_staging"] = kdl.Frame(front, kdl.Vector(0.4, -0.2, 1.0 if name == "fridge" else 0.95))
            # Store all KDL targets and arc duration together for the job to read.
            self.blackboard.set(self.output_key, targets)
            # Also store ROS Pose values at output_key/target for MOVES and MOVEPT.
            for name, value in targets.items():
                # Duration is metadata, not a pose; it stays in the parent target dictionary.
                if name in ("door_open_arc_seconds", "door_close_arc_seconds"):
                    continue
                key = self.output_key + "/" + name
                self.blackboard.register_key(key, access=py_trees.common.Access.WRITE)
                self.blackboard.set(key, [KDLframe2Pose(frame) for frame in value]
                                    if isinstance(value, list) else KDLframe2Pose(value))
        except Exception as error:
            # Fail this subtree on missing transforms or invalid task data; do not start motion.
            self.feedback_message = str(error)
            self.scene.node.get_logger().error(str(error))
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS
