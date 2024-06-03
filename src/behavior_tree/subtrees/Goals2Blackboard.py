# Standard imports
import json

# Third-party imports
import py_trees
from py_trees_ros import subscribers

# ROS imports
import rospy
import actionlib
import std_msgs.msg as std_msgs
from actionlib_msgs.msg import GoalStatus

# Local imports
from riro_navigation.msg import TaskPlanAction, TaskPlanResult, TaskPlanFeedback, GoalInfo, GoalsInfo
from riro_navigation.msg import Dictlistfloat
from riro_navigation.srv import LTLPlan, LTLPlanRequest

class Goals2Blackboard:
    def __init__(self):
        pass

    def update(self):
        pass