"""G1 standing-mode and dual-arm zero-pose subtree."""

from __future__ import annotations

import py_trees

from behavior_tree.subtrees import G1Locomotion, MoveParallel, Policy


STAND_TRANSITION_DEADLINE_SECONDS = 30.0


def create_subtree(
    action_clients,
    robot_names,
    locomotion_client,
    locomotion_status_topic,
    timeout,
    name="G1StandZero",
):
    """Request standing mode, then move both G1 arms to zero in parallel."""
    timeout = float(timeout)
    hold_stand = G1Locomotion.LocomotionCommand(
        name="G1HoldStand",
        action_type="holdStand",
        timeout=STAND_TRANSITION_DEADLINE_SECONDS,
        command_client=locomotion_client,
        goal_status_topic=locomotion_status_topic,
    )
    arm_commands = [
        Policy.MOVEBYPOLICY(
            name=f"Zero_{robot_name}",
            action_client=action_clients[robot_name],
            action_goal={
                "primitive_action": "policy_execute",
                "skill_id": "g1_joint_target",
                "joint_positions": [0.0] * 7,
                "timeout": timeout,
            },
            timeout=timeout,
            robot_name=robot_name,
        )
        for robot_name in robot_names
    ]
    move_arms = MoveParallel.MoveParallel(
        name="G1ZeroArms", children=arm_commands
    )
    root = py_trees.composites.Sequence(name=name, memory=True)
    root.add_children([hold_stand, move_arms])
    return root
