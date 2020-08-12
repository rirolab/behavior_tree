cmake_minimum_required(VERSION 2.8.3)
project(behavior_tree)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES behavior_tree
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

