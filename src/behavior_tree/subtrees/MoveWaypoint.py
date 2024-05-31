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

    def __init__(self, name):
        # Initialization
        super(MOVEG, self).__init__(name=name)
        self.name = name
        self.goal = None
        
    def setup(self, timeout):
        rospy.loginfo(f"[Subtree] MOVEG : setup() called ({self.name}).")
        self.logger.debug("{0}.setup()".format(self.__class__.__name__))

        self.feedback_message = "{}: setup".format(self.name)

        # ROS service
        rospy.wait_for_service('/manage_map/get_region_goal')
        self.getregiongoal_client = rospy.ServiceProxy('/manage_map/get_region_goal', getRegionGoal)

        # ROS action client
        self.nav_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

        return True

    def initialise(self):
        rospy.loginfo('[subtree] MOVEG: initialize() called.')
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))

        blackboard = py_trees.Blackboard()
        self.goal = blackboard.get('goal')
        
    def update(self):
        rospy.loginfo('[subtree] MOVEG: update() called.')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        blackboard = py_trees.Blackboard()

    def terminate(self, new_status):
        pass
