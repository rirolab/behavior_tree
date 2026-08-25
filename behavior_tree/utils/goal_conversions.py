import geometry_msgs


def behavior_to_pose_goal(behavior):
    """
    Resolve a behavior's inline or blackboard pose action goal.
    """
    # Fetch the pose from the blackboard only when the action stores a key.
    pose_ref = behavior.action_goal['pose']
    if type(pose_ref) is geometry_msgs.msg.Pose:
        pose = pose_ref
    else:
        pose = behavior.blackboard.get(pose_ref)

    # Preserve the Cartesian pose fields expected by trajectory_manager.
    return {
        'x': pose.position.x,
        'y': pose.position.y,
        'z': pose.position.z,
        'qx': pose.orientation.x,
        'qy': pose.orientation.y,
        'qz': pose.orientation.z,
        'qw': pose.orientation.w,
    }
