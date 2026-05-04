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

    def initialise(self):
        super(POSE_ESTIMATOR, self).initialise()
        for blackboard in self.robot_blackboards.values():
            blackboard.register_key(
                key=self.name + "/regrasp_target_up",
                access=py_trees.common.Access.WRITE,
            )
            blackboard.register_key(
                key=self.name + "/regrasp_target_down",
                access=py_trees.common.Access.WRITE,
            )
            blackboard.set(self.name + "/regrasp_target_up", Pose())
            blackboard.set(self.name + "/regrasp_target_down", Pose())

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if not self.sent_goal:

            # Request the ring target poses from the world model 
            try:
                regrasp_target_up_world = self.request_named_pose("regrasp_target_up")
                regrasp_target_down_world = self.request_named_pose("regrasp_target_down")
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

                # Left robot: hold the ring vertically in target_up pose, Right robot: grasp the ring in target_down pose
                # if robot_name == self.holding_robot and robot_name is not None and "left" in robot_name:
                #     regrasp_target_up = self.rotate_pose_local(
                #         regrasp_target_up,
                #         axis="z",
                #         angle=np.pi / 2.0,
                #         tool_offset_z=grasp_offset_z,
                #     )
                if robot_name == self.approach_robot and robot_name is not None and "right" in robot_name:
                    regrasp_target_down = self.rotate_pose_local(
                        regrasp_target_down,
                        axis="x",
                        angle=np.pi / 2.0,
                        tool_offset_z=grasp_offset_z,
                    )

                # Right robot: hold the ring vertically in target_up pose, Left robot: grasp the ring in target_down pose
                # if robot_name == self.holding_robot and robot_name is not None and "right" in robot_name:
                #     regrasp_target_up = self.rotate_pose_local(
                #         regrasp_target_up,
                #         axis="z",
                #         angle=-np.pi / 2.0,
                #         tool_offset_z=grasp_offset_z,
                #     )
                if robot_name == self.approach_robot and robot_name is not None and "left" in robot_name:
                    regrasp_target_down = self.rotate_pose_local(
                        regrasp_target_down,
                        axis="x",
                        angle=-np.pi / 2.0,
                        tool_offset_z=grasp_offset_z,
                    )

                blackboard.set(self.name + "/regrasp_target_up", regrasp_target_up)
                blackboard.set(self.name + "/regrasp_target_down", regrasp_target_down)

            self.sent_goal = True
            self.feedback_message = "RingWorldModel: successful pose estimation"
            return py_trees.common.Status.SUCCESS

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
