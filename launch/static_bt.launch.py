#!/usr/bin/python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription,
                                SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution, LaunchConfiguration
import xacro
import yaml

def generate_launch_description():

    # Get the launch directory
    bt_dir = get_package_share_directory('behavior_tree')

    params_file=os.path.join(
        bt_dir, 'params', 'default.yaml')
    
    bt_node = Node(
        name="tree",
        package='behavior_tree',
        executable='static_bt',
        output='screen',
        emulate_tty=True,
        arguments=[],
        parameters=[params_file]
        )
    
    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        bt_node, 
    ])
