

# Behavior Tree
A customized bahavior tree package for mobile manipulators from Prof. Daehyung Park.

## Publishers

## Subscribers
* symbol_grounding (std_msgs.String)

## Service Clients
* get_object_pose (String_Pose)
* get_object_grasp_pose (String_Pose)
* get_object_height (String_Pose)
* get_object_rnd_pose (String_Pose)
* get_object_close_pose (String_Pose)
* arm_client/command (complex_action_client.srv.String_Int)
* remove_wm_object

## Parameters
Following ROS parameters need to be set before running this package:

* world_frame: (default: /base_footprint)
* arm_base_frame: (default: /ur_arm_base_link)
* grasp_offset_z: (default: 0.02)
* top_offset_z: (default: 0.15)
* gripper_open_pos
* gripper_close_pos
* gripper_open_force
* gripper_close_force
* init_config: A list joint angles to initialize before running subtrees (default: [0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.])

## Installation
sudo apt-get install ros-melodic-rqt-reconfigure


This package is under development...

## Test
You can visualize the tree using terminal:
```bash
py-trees-tree-watcher
```
or using GUI:
```bash
rqt_py_trees
```


You can test a static behavior tree:
```bash
ros2 launch ur5_demo ur5_robotiq_grasping_demo.launch.py
```
```bash
ros2 launch behavior_tree static_bt.launch.py
```
```bash
ros2 topic pub /move_cmd std_msgs/msg/Empty
```


## Note
Arm client service names must match with the names in subtrees' service names:
```bash
arm_client/command
arm_client/status
```