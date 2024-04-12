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
    print("Select either simulation or real setup:")
    sim = True
    while (sim == True):
        print("     + Option N1: UR5 ROBOT (REAL).")
        print("     + Option N2: UR5 ROBOT (SIMULATION).")
        robot_config = input ("  Please select: ")
        if (robot_config == "1"):
            sim = False
        elif (robot_config == "2"):
            sim = True
        else:
            print ("  Please select a valid option!")
    print("")

    # Get the launch directory
    bt_dir = get_package_share_directory('behavior_tree')

    if sim:
        params_file=os.path.join(
            bt_dir, 'params', 'default.yaml')
    else:
        params_file=os.path.join(
            bt_dir, 'params', 'real_default.yaml')

    bt_node = Node(
        name="tree",
        package='behavior_tree',
        executable='dynamic_bt',
        output='screen',
        emulate_tty=True,
        arguments=[],
        parameters=[params_file]
        )
    
    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        bt_node, 
    ])

    


## <?xml version="1.0" encoding="utf-8"?>
## <launch>

##   <arg name="sim" default="false" />
##   <arg name="topic_json" default="$(find behavior_tree)/data/default.json" />

##   <group if="$(arg sim)">
##     <node name="action_client"
##           pkg="complex_action_client"
##           type="arm_client_ur5.py"
##           args="--sim"
##           output="screen"
##           required="true" />
##   </group>
##   <group unless="$(arg sim)">
##     <node name="action_client"
##           pkg="complex_action_client"
##           type="arm_client_ur5.py"
##           args=""
##           output="screen"
##           required="true" />
##   </group>

##   <node name="behavior_tree"
##         pkg="behavior_tree"
##         type="dynamic_behavior_tree.py"
##         args="-s $(arg sim) -t $(arg topic_json)"
##         output="screen"
##         required="true" />
      
## </launch>

