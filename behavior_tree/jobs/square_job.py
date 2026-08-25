import json

import py_trees
from geometry_msgs.msg import Point, Pose, Quaternion
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import MoveBlend
from behavior_tree.subtrees import MovePose
from behavior_tree.utils.validation_utils import StepValidationResult

# Previous full-size square waypoints.
# SQUARE_POSITIONS = [
#     [0.333, -0.233, 0.513],
#     [0.333, 0.233, 0.513],
#     [0.626, 0.233, 0.513],
#     [0.626, -0.233, 0.513],
#     [0.333, -0.233, 0.513],
# ]
# Half-size square waypoints.
SQUARE_POSITIONS = [
    [0.40625, -0.1165, 0.513],
    [0.40625, 0.1165, 0.513],
    [0.55275, 0.1165, 0.513],
    [0.55275, -0.1165, 0.513],
    [0.40625, -0.1165, 0.513],
]
# Use the world-frame XYZW quaternion for every square waypoint.
SQUARE_ORIENTATION = [0.0, 0.0, 0.0, 1.0]
SQUARE_TIMEOUT = 1.0


class Move(base_job.BaseJob):
    """
    A job handler that moves the end-effector through the fixed FR3 square path.
    """

    def __init__(self, node):
        """
        Subscribe to grounding messages and preload blackboard parameters.
        """
        super(Move, self).__init__(node)
        self.init_blackboard_parameters()

    def acceptable_step(self, step):
        """
        Check whether this job should accept a square primitive action.
        """
        # Accept only the fixed single-arm square command.
        if step.get("primitive_action") != "square":
            return False
        elif not self.check_robot_count(step, num_robot_required=1):
            return False
        else:
            return True

    def validate_step(self, step):
        """
        Validate whether the square step can build the fixed path.
        """
        # Ignore steps owned by other jobs.
        if not self.acceptable_step(step):
            return StepValidationResult.NOT_APPLICABLE

        # Reject invalid blend timing before the subtree is created.
        if "blend_duration" in step:
            try:
                blend_duration = float(step["blend_duration"])
                timeout = float(step.get("timeout", SQUARE_TIMEOUT))
            except (TypeError, ValueError):
                self._node.get_logger().warning(
                    "square_job: blend_duration and timeout must be numeric"
                )
                return StepValidationResult.REJECT_GOAL
            if blend_duration <= 0.0 or timeout <= 0.0 or blend_duration >= timeout:
                self._node.get_logger().warning(
                    "square_job: blend_duration must be positive and shorter than timeout"
                )
                return StepValidationResult.REJECT_GOAL

        return StepValidationResult.ACCEPT_GOAL

    def incoming(self, msg):
        """
        Store incoming square goals until the dynamic tree consumes them.
        """
        # Reject overlapping square goals while another one is pending.
        if self.goal:
            self._node.get_logger().error(
                "square_job: rejecting new goal, previous still in the pipeline"
            )
        else:
            grounding = json.loads(msg.data)["params"]
            for i in range(len(grounding.keys())):
                step = grounding.get(str(i + 1))
                if step is None:
                    continue
                validation_result = self.validate_step(step)
                if validation_result == StepValidationResult.REJECT_GOAL:
                    self._node.get_logger().error(
                        "square_job: rejecting invalid square goal"
                    )
                    continue
                if validation_result == StepValidationResult.ACCEPT_GOAL:
                    self.goal = grounding
                    break

    def create_root(
        self,
        action_client,
        idx="1",
        goal=std_msgs.Empty(),
        robot_name=None,
        **kwargs,
    ):
        """
        Create a straight-line pose sequence for the fixed square path.
        """
        # Ignore steps owned by other jobs.
        if not self.acceptable_step(goal[idx]):
            return None

        # Write fixed square poses to the same blackboard namespace used by MovePose.
        timeout = float(goal[idx].get("timeout", SQUARE_TIMEOUT))
        blend_duration = goal[idx].get("blend_duration")
        enable_blend = blend_duration is not None
        if enable_blend:
            blend_duration = float(blend_duration)
        blackboard = py_trees.blackboard.Client(namespace=robot_name)
        square = py_trees.composites.Sequence(name="Square", memory=True)
        blend_children = []
        for i, position in enumerate(SQUARE_POSITIONS, start=1):
            pose_key = f"Square{idx}/pose{i}"
            pose = Pose(
                position=Point(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(
                    x=SQUARE_ORIENTATION[0],
                    y=SQUARE_ORIENTATION[1],
                    z=SQUARE_ORIENTATION[2],
                    w=SQUARE_ORIENTATION[3],
                ),
            )
            blackboard.register_key(key=pose_key, access=py_trees.common.Access.WRITE)
            blackboard.set(pose_key, pose)

            # Build blend children with Cartesian straight-line pose commands.
            if enable_blend:
                blend_children.append(
                    MovePose.MOVES(
                        name=f"Square{i}",
                        action_client=action_client,
                        action_goal={"pose": pose_key},
                        timeout=timeout,
                        robot_name=robot_name,
                    )
                )
                continue

            # Move each segment with Cartesian straight-line pose commands.
            square.add_child(
                MovePose.MOVES(
                    name=f"Square{i}",
                    action_client=action_client,
                    action_goal={"pose": pose_key},
                    timeout=timeout,
                    robot_name=robot_name,
                )
            )

        # Dispatch the square through complex action client blending while running.
        if enable_blend:
            square = MoveBlend.MoveBlend(
                name="BlendSquare",
                action_client=action_client,
                timeout=timeout * len(blend_children),
                blend_duration=blend_duration,
                robot_name=robot_name,
                children=blend_children,
            )

        # Repeat the full square loop when the payload asks for it.
        if goal[idx].get("repeat", False):
            return py_trees.decorators.Repeat(
                name="RepeatSquare",
                child=square,
                num_success=-1,
            )

        return square
