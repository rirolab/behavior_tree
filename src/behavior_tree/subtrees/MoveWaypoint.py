# Standard imports
import json
import re

# ROS2 imports
import rospy
import rosparam
import py_trees_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# RIRO imports
from riro_navigation.msg import TaskPlanAction, TaskPlanResult
from riro_navigation.srv import getRegionGoal
from riro_navigation.srv import NavigationControl, NavigationControlResponse

# Other imports 
import numpy as np
import py_trees

class MOVEG(py_trees.behaviour.Behaviour):
    """
    Move to Goal
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self):
        self.goal = None
        pass

    def setup(self, timeout):
        pass

    def initialise(self):
        pass

    def update(self):
        pass

    def terminate(self, new_status):
        pass
