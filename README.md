

# Behavior Tree
A customized behavior tree package for (mobile) manipulators from Prof. Daehyung Park.

## Entrypoints
* `dynamic_bt` — single-arm dynamic BT
* `multi_dynamic_bt` — multi-arm BT (dispatches grounding steps to one or more robot namespaces)
* `static_bt` — legacy static BT

## Subscribers
* `symbol_grounding` (std_msgs/String)
* `{robot}/arm_client/arm/goal_status` (action_msgs/GoalStatus) — arm action status, channel-split
* `{robot}/arm_client/gripper/goal_status` (action_msgs/GoalStatus) — gripper action status, channel-split

## Service Clients
* `get_object_pose` (riro_srvs/StringPose)
* `get_object_grasp_pose` (riro_srvs/StringPose)
* `get_object_height` (riro_srvs/StringPose)
* `get_object_rnd_pose` (riro_srvs/StringPose)
* `get_object_close_pose` (riro_srvs/StringPose)
* `{robot}/arm_client/command` (riro_srvs/StringGoalStatus)
* `remove_wm_object`

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

## Run

Single-arm dynamic BT (paired with one `arm_client`):
```bash
ros2 launch behavior_tree dynamic_bt.launch.py
```

Multi-arm BT (paired with N `arm_client` nodes, one per robot namespace listed in YAML):
```bash
ros2 launch behavior_tree multi_dynamic_bt.launch.py params_file:=/path/to/your.yaml
```

The `params_file` LaunchArg accepts any YAML defining `robot:` plus per-robot params (see `params/multi_default.yaml`).

## Visualize

```bash
py-trees-tree-watcher    # terminal
rqt_py_trees             # GUI
```

## Note
Arm client service / topic names must match between this package and `complex_action_client`:
```
{robot}/arm_client/command
{robot}/arm_client/arm/goal_status
{robot}/arm_client/gripper/goal_status
```