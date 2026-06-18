import copy
import numpy as np
import PyKDL
import py_trees
import rclpy
from geometry_msgs.msg import Pose
from riro_srvs.srv import StringPose

from complex_action_client import misc

from . import WorldModel


class POSE_ESTIMATOR(WorldModel.POSE_ESTIMATOR):
    """
    Ring-specific world model pose estimator.

    This keeps the standard WorldModel behaviour and also publishes
    regrasp target poses for the holding/approach robots.
    """

    def __init__(self, name, object_dict=None, en_random=False, en_close_pose=False, **kwargs):
        self.holding_robot = kwargs.get("holding_robot")
        self.approach_robot = kwargs.get("approach_robot")
        super(POSE_ESTIMATOR, self).__init__(
            name=name,
            object_dict=object_dict,
            en_random=en_random,
            en_close_pose=en_close_pose,
            **kwargs,
        )

    def setup(self, node=None, timeout=py_trees.common.Duration.INFINITE):
        # Initialize parent world-model clients before reading node parameters.
        super(POSE_ESTIMATOR, self).setup(node=node, timeout=timeout)

        # Cache the BT runtime mode for sim/real pose selection.
        self.is_sim = bool(self.node.get_parameter("sim").value)
        return True

    def initialise(self):
        super(POSE_ESTIMATOR, self).initialise()
        BLACKBOARD_POSE_KEYS = [
            "regrasp_target_up",
            "regrasp_target_down",
            "regrasp_target_down_left",
            "regrasp_target_down_right",
            "regrasp_target_down_half_left",
            "real_regrasp_target_down_half_left",
            "horizontal_grasp_top_right",
            "horizontal_grasp_top_left",
            "horizontal_grasp_top_right_wp1",
            "horizontal_grasp_top_right_wp2",
            "horizontal_grasp_top_left_wp2",
        ]
        for blackboard in self.robot_blackboards.values():
            for pose_key in BLACKBOARD_POSE_KEYS:
                full_key = f"{self.name}/{pose_key}"
                blackboard.register_key(
                    key=full_key,
                    access=py_trees.common.Access.WRITE,
                )
                blackboard.set(full_key, Pose())

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if not self.sent_goal:

            # Get object name from the grounding
            obj = self.object_dict.get('target')
            
            if obj == 'dual_grasp_target':
                # Request the ring target poses from the world model 
                try:
                    regrasp_target_up_world = self.request_named_pose("regrasp_target_up")
                    regrasp_target_down_world = self.request_named_pose("regrasp_target_down")
                    horizontal_grasp_top_right_world = self.request_named_pose("horizontal_grasp_top_right")
                    horizontal_grasp_top_left_world = self.request_named_pose("horizontal_grasp_top_left")
                except Exception as e:
                    self.feedback_message = "RingWorldModel: pose service is not available: %s" % e
                    return py_trees.common.Status.FAILURE

                # Convert the world-frame poses to each robot's local frame and write to blackboard
                for robot_name, blackboard in self.robot_blackboards.items():
                    arm_base_frame = self._arm_base_frames[robot_name]
                    base2arm_baselink = self.lookup_base_frame(arm_base_frame)
                    grasp_offset_z = self.grasp_offset_z_by_robot[robot_name]
                    regrasp_target_up = copy.deepcopy(self.get_grasp_pose(regrasp_target_up_world, base2arm_baselink, grasp_offset_z))
                    regrasp_target_down = copy.deepcopy(self.get_grasp_pose(regrasp_target_down_world, base2arm_baselink, grasp_offset_z))
                    horizontal_grasp_top_right = copy.deepcopy(
                        self.get_grasp_pose(horizontal_grasp_top_right_world, base2arm_baselink, grasp_offset_z)
                    )
                    horizontal_grasp_top_left = copy.deepcopy(
                        self.get_grasp_pose(horizontal_grasp_top_left_world, base2arm_baselink, grasp_offset_z)
                    )

                    # Left robot: hold the ring vertically in target_up pose, Right robot: grasp the ring in target_down pose
                    if robot_name == self.approach_robot and robot_name is not None and "right" in robot_name:
                        regrasp_target_down = self.rotate_pose_local(
                            regrasp_target_down,
                            axis="x",
                            angle=np.pi / 2.0,
                            tool_offset_z=grasp_offset_z,
                        )

                    # Right robot: hold the ring vertically in target_up pose, Left robot: grasp the ring in target_down pose
                    if robot_name == self.approach_robot and robot_name is not None and "left" in robot_name:
                        regrasp_target_down = self.rotate_pose_local(
                            regrasp_target_down,
                            axis="x",
                            angle=-np.pi / 2.0,
                            tool_offset_z=grasp_offset_z,
                        )
                        raise NotImplementedError("TODO")

                    # Define regrasp target down right/left pose
                    regrasp_target_down_right = copy.deepcopy(regrasp_target_down) 
                    regrasp_target_down_right.position.y -= 0.05
                    regrasp_target_down_left = copy.deepcopy(regrasp_target_down) 
                    regrasp_target_down_left.position.x -= 0.1 # frame changed?
                    regrasp_target_down_left.position.y += 0.05
                    regrasp_target_down_half_left = copy.deepcopy(regrasp_target_down) 
                    regrasp_target_down_half_left.position.x -= 0.1 # frame changed?
                    regrasp_target_down_half_left.position.y += 0.01
                    real_regrasp_target_down_half_left = copy.deepcopy(regrasp_target_down) 
                    real_regrasp_target_down_half_left.position.y += 0.025

                    # Define horizontal grasp top waypoint poses 
                    horizontal_grasp_top_right_wp1 = copy.deepcopy(horizontal_grasp_top_right)
                    horizontal_grasp_top_right_wp1.position.z = (regrasp_target_down.position.z + horizontal_grasp_top_right.position.z) / 2.0
                    horizontal_grasp_top_right_wp1 = self.rotate_pose_local(
                            horizontal_grasp_top_right_wp1,
                            axis="x",
                            angle=np.pi / 4.0,
                            tool_offset_z=grasp_offset_z,
                        )
                    
                    # TODO: horizontal_grasp_top_left_wp1
                    horizontal_grasp_top_right_wp2 = copy.deepcopy(horizontal_grasp_top_right) 
                    horizontal_grasp_top_right_wp2.position.x = regrasp_target_up.position.x
                    horizontal_grasp_top_left_wp2 = copy.deepcopy(horizontal_grasp_top_left) 
                    horizontal_grasp_top_left_wp2.position.x = regrasp_target_up.position.x

                    # Set regrasp target poses
                    blackboard.set(self.name + "/regrasp_target_up", regrasp_target_up)
                    blackboard.set(self.name + "/regrasp_target_down", regrasp_target_down)
                    blackboard.set(self.name + "/regrasp_target_down_right", regrasp_target_down_right)
                    blackboard.set(self.name + "/regrasp_target_down_left", regrasp_target_down_left)
                    blackboard.set(self.name + "/regrasp_target_down_half_left", regrasp_target_down_half_left)
                    blackboard.set(self.name + "/real_regrasp_target_down_half_left", real_regrasp_target_down_half_left)
                    # Set horizontal grasp top poses
                    blackboard.set(self.name + "/horizontal_grasp_top_right", horizontal_grasp_top_right)
                    blackboard.set(self.name + "/horizontal_grasp_top_left", horizontal_grasp_top_left)
                    blackboard.set(self.name + "/horizontal_grasp_top_right_wp1", horizontal_grasp_top_right_wp1)
                    blackboard.set(self.name + "/horizontal_grasp_top_right_wp2", horizontal_grasp_top_right_wp2)
                    blackboard.set(self.name + "/horizontal_grasp_top_left_wp2", horizontal_grasp_top_left_wp2)

                self.sent_goal = True
                self.feedback_message = "RingWorldModel: successful pose estimation"
                return py_trees.common.Status.SUCCESS
            
            else:
                pass

        self.feedback_message = "RingWorldModel: successful pose estimation"
        return py_trees.common.Status.SUCCESS

    def request_named_pose(self, pose_name):
        """
        Request a named pose from the world-model pose service.

        Args:
            pose_name (:obj:`str`): name of the target pose to query.

        Returns:
            :class:`geometry_msgs.msg.Pose`: raw world-frame pose from the service.
        """
        req = StringPose.Request(data=pose_name)
        future = self.grasp_pose_srv_req.call_async(req)
        while rclpy.ok():
            if future.done():
                break
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return future.result().pose

    @staticmethod
    def rotate_pose_local(pose, axis, angle, tool_offset_z=0.0):
        """
        Rotate a pose around its local axis while keeping the tool tip fixed.

        Args:
            pose (:class:`geometry_msgs.msg.Pose`): source pose.
            axis (:obj:`str`): one of ``x`` or ``z``.
            angle (:obj:`float`): rotation angle in radians.
            tool_offset_z (:obj:`float`): local z offset from the robot ee frame
                origin to the tool tip that should stay fixed during rotation.

        Returns:
            :class:`geometry_msgs.msg.Pose`: rotated pose.
        """
        frame = misc.pose2KDLframe(pose)
        tip_local = PyKDL.Vector(0.0, 0.0, tool_offset_z)
        tip_world = frame * tip_local

        if axis == "x":
            rotated = frame.M * PyKDL.Rotation.RotX(angle)
        elif axis == "z":
            rotated = frame.M * PyKDL.Rotation.RotZ(angle)
        else:
            raise ValueError(f"unsupported axis: {axis}")

        frame.M = rotated
        frame.p = tip_world - rotated * tip_local
        return misc.KDLframe2Pose(frame)
