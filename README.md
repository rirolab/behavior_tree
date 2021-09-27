

# Behavior Tree
A customized bahavior tree package for mobile manipulators from Prof. Daehyung Park.


## Parameters
Following ROS parameters need to be set before running this package:

* init_config A list joint angles to initialize before running subtrees 



## Installation
sudo apt-get install ros-kinetic-rqt-py-trees ros-kinetic-rqt-reconfigure


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



## Note
Arm client service names must match with the names in subtrees' service names:
```bash
arm_client/command
arm_client/status
```