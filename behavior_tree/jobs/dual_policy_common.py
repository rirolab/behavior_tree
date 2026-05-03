import py_trees
from geometry_msgs.msg import Pose

from behavior_tree.subtrees import MoveJoint, MovePose, Policy, WorldModel
from behavior_tree.utils.parameter_utils import make_string_list


MOTION_PARAMETER_KEYS = [
    "init_config",
    "init_pose",
    "init_together",
]


def robot_names_from_command(command):
    """
    Read the robot list from one dual-arm grounding command.

    Args:
        command (:obj:`dict`): incoming grounding step.

    Returns:
        [:obj:`str`]: robot names requested by the command.
    """
    return make_string_list(command.get("robot", []))


def make_parallel(name, robots, child_factory):
    """
    Create a parallel subtree with one child per robot.

    Args:
        name (:obj:`str`): name for the parallel subtree.
        robots ([:obj:`str`]): robot names that should run together.
        child_factory (callable): function that creates one child for a robot.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: parallel subtree root.
    """
    root = py_trees.composites.Parallel(
        name=name,
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False,
        ),
    )
    root.add_children([child_factory(robot_name) for robot_name in robots])
    return root


def make_init_joint_parallel(name, action_clients, robots, timeout=3.0):
    """
    Create a parallel subtree that moves all robots to their initial joints.

    Args:
        name (:obj:`str`): name for the parallel subtree.
        action_clients (:obj:`dict`): action clients keyed by robot name.
        robots ([:obj:`str`]): robot names to initialise.
        timeout (:obj:`float`): command timeout in seconds.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: parallel subtree root.
    """
    def make_child(robot_name):
        """
        Create one initial joint motion child for a robot.

        Args:
            robot_name (:obj:`str`): robot namespace for this child.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: joint motion behaviour.
        """
        init_config = read_robot_parameter(robot_name, "init_config")
        if not init_config:
            init_config = read_robot_parameter(robot_name, "init_pose")
        return MoveJoint.MOVEJ(
            name=f"{robot_name}_Init",
            action_client=action_clients[robot_name],
            action_goal=init_config,
            timeout=timeout,
            robot_name=robot_name,
        )

    return make_parallel(name, robots, make_child)


def make_blackboard_pose_parallel(
    name,
    action_clients,
    robots,
    pose_key,
    motion_type="move_pose",
    timeout=3.0,
):
    """
    Create a parallel subtree that moves robots to a blackboard pose.

    Args:
        name (:obj:`str`): name for the parallel subtree.
        action_clients (:obj:`dict`): action clients keyed by robot name.
        robots ([:obj:`str`]): robot names to move.
        pose_key (:obj:`str`): blackboard key that stores the target pose.
        motion_type (:obj:`str`): pose motion type, either pose or straight.
        timeout (:obj:`float`): command timeout in seconds.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: parallel subtree root.
    """
    motion_class = MovePose.MOVES if motion_type == "move_straight" else MovePose.MOVEP

    def make_child(robot_name):
        """
        Create one blackboard pose motion child for a robot.

        Args:
            robot_name (:obj:`str`): robot namespace for this child.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: pose motion behaviour.
        """
        return motion_class(
            name=f"{robot_name}_{name}",
            action_client=action_clients[robot_name],
            action_goal={"pose": pose_key},
            timeout=timeout,
            robot_name=robot_name,
        )

    return make_parallel(name, robots, make_child)


def make_parameter_pose_parallel(
    name,
    action_clients,
    robots,
    parameter_name,
    motion_type="move_pose",
    timeout=3.0,
):
    """
    Create a parallel subtree that moves robots to a configured pose parameter.

    Args:
        name (:obj:`str`): name for the parallel subtree.
        action_clients (:obj:`dict`): action clients keyed by robot name.
        robots ([:obj:`str`]): robot names to move.
        parameter_name (:obj:`str`): blackboard parameter that stores the pose.
        motion_type (:obj:`str`): pose motion type, either pose or straight.
        timeout (:obj:`float`): command timeout in seconds.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: parallel subtree root.
    """
    motion_class = MovePose.MOVES if motion_type == "move_straight" else MovePose.MOVEP

    def make_child(robot_name):
        """
        Create one parameter pose motion child for a robot.

        Args:
            robot_name (:obj:`str`): robot namespace for this child.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: pose motion behaviour.
        """
        return motion_class(
            name=f"{robot_name}_{name}",
            action_client=action_clients[robot_name],
            action_goal={"pose": pose_from_parameter(read_robot_parameter(robot_name, parameter_name))},
            timeout=timeout,
            robot_name=robot_name,
        )

    return make_parallel(name, robots, make_child)


def make_policy_parallel(name, action_clients, robots, command, timeout=5.0):
    """
    Create a parallel subtree that executes the same policy on all robots.

    Args:
        name (:obj:`str`): name for the parallel subtree.
        action_clients (:obj:`dict`): action clients keyed by robot name.
        robots ([:obj:`str`]): robot names to command.
        command (:obj:`dict`): policy command from the grounding step.
        timeout (:obj:`float`): command timeout in seconds.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: parallel subtree root.
    """
    def make_child(robot_name):
        """
        Create one policy execution child for a robot.

        Args:
            robot_name (:obj:`str`): robot namespace for this child.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: policy behaviour.
        """
        return Policy.MOVEBYPOLICY(
            name=f"{robot_name}_MoveByPolicy",
            action_client=action_clients[robot_name],
            action_goal=command,
            timeout=timeout,
            robot_name=robot_name,
        )

    return make_parallel(name, robots, make_child)


def make_pose_estimator(name, command, robots, tf_buffer, include_destination=False):
    """
    Create a world-model pose estimator for the shared dual-arm target.

    Args:
        name (:obj:`str`): name for the estimator subtree.
        command (:obj:`dict`): grounding command with object and destination data.
        robots ([:obj:`str`]): robot names used by the estimator.
        tf_buffer (:class:`~tf2_ros.buffer.Buffer`): TF buffer for pose lookup.
        include_destination (:obj:`bool`): whether to require a destination.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: pose estimator behaviour.
    """
    target = first_present(command, ["object", "obj", "target", "held_object"])
    if target is None:
        raise RuntimeError(f"{name}: object is required")

    object_dict = {"target": target}
    if include_destination:
        destination = first_present(command, ["destination", "dest"])
        if destination is None:
            raise RuntimeError(f"{name}: destination is required")
        object_dict["destination"] = destination
        if "destination_offset" in command:
            object_dict["destination_offset"] = command["destination_offset"]

    return WorldModel.POSE_ESTIMATOR(
        name=name,
        object_dict=object_dict,
        tf_buffer=tf_buffer,
        robot_names=robots,
    )


def read_robot_parameter(robot_name, parameter_name):
    """
    Read one namespaced parameter from the blackboard.

    Args:
        robot_name (:obj:`str`): robot namespace.
        parameter_name (:obj:`str`): blackboard parameter name.

    Returns:
        configured blackboard value for the robot.
    """
    blackboard = py_trees.blackboard.Client(namespace=robot_name)
    blackboard.register_key(key=parameter_name, access=py_trees.common.Access.READ)
    return blackboard.get(parameter_name)


def pose_from_parameter(value):
    """
    Convert a configured pose value into a ROS Pose message.

    Args:
        value: pose value as a Pose, dict, or list.

    Returns:
        :class:`~geometry_msgs.msg.Pose`: pose message.
    """
    if isinstance(value, Pose):
        return value
    if isinstance(value, dict):
        return pose_from_dict(value)
    if isinstance(value, (list, tuple)) and len(value) == 7:
        return pose_from_list(value)
    raise ValueError("pose parameter must be a Pose, dict, or [x, y, z, qx, qy, qz, qw]")


def pose_from_dict(value):
    """
    Convert a pose dictionary into a ROS Pose message.

    Args:
        value (:obj:`dict`): pose dictionary.

    Returns:
        :class:`~geometry_msgs.msg.Pose`: pose message.
    """
    pose = Pose()
    pose.position.x = float(value["x"])
    pose.position.y = float(value["y"])
    pose.position.z = float(value["z"])
    pose.orientation.x = float(value.get("qx", value.get("orientation_x", 0.0)))
    pose.orientation.y = float(value.get("qy", value.get("orientation_y", 0.0)))
    pose.orientation.z = float(value.get("qz", value.get("orientation_z", 0.0)))
    pose.orientation.w = float(value.get("qw", value.get("orientation_w", 1.0)))
    return pose


def pose_from_list(value):
    """
    Convert a seven-value pose list into a ROS Pose message.

    Args:
        value (:obj:`list`): [x, y, z, qx, qy, qz, qw] pose values.

    Returns:
        :class:`~geometry_msgs.msg.Pose`: pose message.
    """
    pose = Pose()
    pose.position.x = float(value[0])
    pose.position.y = float(value[1])
    pose.position.z = float(value[2])
    pose.orientation.x = float(value[3])
    pose.orientation.y = float(value[4])
    pose.orientation.z = float(value[5])
    pose.orientation.w = float(value[6])
    return pose


def first_present(values, keys):
    """
    Return the first value present for a list of candidate keys.

    Args:
        values (:obj:`dict`): source dictionary.
        keys ([:obj:`str`]): candidate keys in priority order.

    Returns:
        first matching value or :obj:`None`.
    """
    for key in keys:
        if key in values:
            return values[key]
    return None
