#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Create the launch description for the multi-robot dynamic behavior tree.

    Returns:
        :class:`~launch.LaunchDescription`: launch description for the tree node.
    """
    bt_dir = get_package_share_directory('behavior_tree')
    params_file = os.path.join(
        bt_dir,
        'params',
        'multi_default.yaml',
    )

    bt_node = Node(
        name="tree",
        package='behavior_tree',
        executable='multi_dynamic_bt',
        output='screen',
        emulate_tty=True,
        arguments=[],
        parameters=[params_file],
    )

    return LaunchDescription([
        bt_node,
    ])
