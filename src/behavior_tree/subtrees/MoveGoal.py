#!/usr/bin/env python3

# standard imports
import re
import json

# ROS imports
import rospy
import time
import threading
import actionlib
import py_trees
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

# local imports
from riro_navigation.msg import TaskPlanResult, TaskPlanResultTemp
from riro_navigation.srv import getRegionGoal
import numpy as np

class MOVEG(py_trees.behaviour.Behaviour):
    """
    Move Base
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    # __init__(self) should instantiate the behaviour sufficiently for offline dot graph generation
    # No hardware connections that may not be there, e.g. usb lidars
    # No middleware connections to other software that may not be there, e.g. ROS pubs/subs/services
    # No need to fire up other needlessly heavy resources, e.g. heavy threads in the background
    def __init__(self, name, idx='', action_goal=None, destination=None, sim='true'):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        super(MOVEG, self).__init__(name=name)

        self.idx = idx
        self.action_goal = action_goal # pose
        self.destination = destination  # 'r1', 'r2', etc.
        self.sim = sim

        self.robot_pose = None

        # May be deprecated
        # self.rviz_msg = None

        # Mode
        self.relaxation_set = True
        self.lock = threading.Lock()
        self.result = None
        self.sent_goal = False


    # setup(self) handles all other one-time initialisations of resources that are required for execution:
    # Essentially, all the things that the constructor doesn’t handle - hardware connections, middleware and other heavy resources.
    def setup(self, timeout):
        """
        When is this called?
          This function should be either manually called by your program
          to setup this behaviour alone, or more commonly, via
          :meth:`~py_trees.behaviour.Behaviour.setup_with_descendants`
          or :meth:`~py_trees.trees.BehaviourTree.setup`, both of which
          will iterate over this behaviour, it's children (it's children's
          children ...) calling :meth:`~py_trees.behaviour.Behaviour.setup`
          on each in turn.

          If you have vital initialisation necessary to the success
          execution of your behaviour, put a guard in your
          :meth:`~py_trees.behaviour.Behaviour.initialise` method
          to protect against entry without having been setup.

        What to do here?
          Delayed one-time initialisation that would otherwise interfere
          with offline rendering of this behaviour in a tree to dot graph
          or validation of the behaviour's configuration.

          Good examples include:

          - Hardware or driver initialisation
          - Middleware initialisation (e.g. ROS pubs/subs/services)
          - A parallel checking for a valid policy configuration after
            children have been added or removed
        """
        rospy.loginfo('[subtree] movebase: setup() called.')
        self.feedback_message = "{}: setup".format(self.name)

        blackboard = py_trees.Blackboard()

        # parameter settings
        # if self.sim == 'true' or self.sim == 'True':
        #     self.sim = True
        #     self.map_frame = 'map_carla' 
        #     self.relax_distance_threshold = 10
        #     rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.robot_pose_callback)
        # elif self.sim == 'false' or self.sim == 'False':
        #     self.sim = False
        #     self.map_frame = 'map'
        #     self.relax_distance_threshold = 3
        #     rospy.Subscriber("/odom", Odometry, self.robot_pose_callback)
        # else:
        #     self.sim = False
        #     print("sim arg strange")
        self.sim = True
        self.map_frame = 'map_carla' 
        self.relax_distance_threshold = 10
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.robot_pose_callback)

        # ROS client
        self.nav_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.getregiongoal_client = rospy.ServiceProxy('/manage_map/get_region_goal', getRegionGoal)

        # ROS publihser
        self.nav_status_pub = rospy.Publisher('/status_to_planner', TaskPlanResult, queue_size=10)
        self.blackboard = py_trees.blackboard.Blackboard() # Is this line necessary?

        rospy.loginfo('[subtree] movebase: setup() done.')
        return True

    # initialise(self) configures and resets the behaviour ready for (repeated) execution
    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        rospy.loginfo('[subtree] movebase: initialise() called.')
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        rospy.loginfo(f"{self.__class__.__name__}.intialise() called")

        self.sent_goal = False

    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        rospy.loginfo('[subtree] movebase: update() called.')
        self.logger.debug("%s.update()" % self.__class__.__name__)
        robot_pose = None
        if not self.sent_goal:
            with self.lock:
                robot_pose = self.robot_pose
            if robot_pose is None:
                return py_trees.common.Status.RUNNING
            # Setting the goal
            nav_goal = MoveBaseGoal()

            # Setting MoveBaseGoal()'s header(std_msgs/Header)
            nav_goal.target_pose.header.frame_id = self.map_frame
            nav_goal.target_pose.header.stamp = rospy.Time.now()

            # When the task_plan is format of 'r1,' 'r5,' etc.
            if re.match(r'r\d+', self.destination):
                # Get the goal from service '/manage_map/get_region_goal'
                rospy.wait_for_service('/manage_map/get_region_goal')
                response = self.getregiongoal_client(int(self.destination[1:]))

                nav_goal.target_pose.pose.position.x = response.goal_x
                nav_goal.target_pose.pose.position.y = response.goal_y
            else:
                nav_goal.target_pose.pose.position.x = np.float32(self.action_goal['pose']['x'])
                nav_goal.target_pose.pose.position.y = np.float32(self.action_goal['pose']['y'])
            nav_goal.target_pose.pose.position.z = 0
            
            nav_goal.target_pose.pose.orientation.x = 0
            nav_goal.target_pose.pose.orientation.y = 0
            nav_goal.target_pose.pose.orientation.z = 0
            nav_goal.target_pose.pose.orientation.w = 1

            # if self.sim == False:
            #     rate = rospy.Rate(1)
            #     rospy.loginfo(f"Navigation Plan Ready!")
            #     while self.rviz_msg == None:
            #         rate.sleep()
            
            rospy.loginfo(f"Navigation Plan Being Executed!")

            ######### Existing nav_move_base() code #########

            self.nav_client.send_goal(nav_goal)

            if self.relaxation_set:
                min_robot_goal = self.l1_distance(nav_goal.target_pose.pose.position.x, 
                                                  nav_goal.target_pose.pose.position.y,
                                                  robot_pose.x, 
                                                  robot_pose.y)

            # Initialize the 'result' variable
            result = TaskPlanResult()
            result.relaxation = False
            result.robot_x = 0
            result.robot_y = 0
            result.goal_x = 0
            result.goal_y = 0

            # Wait for 1s for the response from move_base
            while not self.nav_client.wait_for_result(rospy.Duration(1.0)):
                # if self.rviz_msg == "Cancel":
                #     self.nav_client.cancel_goal()
                #     rospy.loginfo(f"Navigation Plan Cancelled!")
                # elif self.rviz_msg == "Pause":
                #     self.nav_client.cancel_goal()
                #     rospy.loginfo(f"Navigation Plan Paused!")

                if self.relaxation_set:
                    # Check relaxation trigger
                    curr_robot_goal = self.l1_distance(nav_goal.target_pose.pose.position.x,
                                                       nav_goal.target_pose.pose.position.y,
                                                       robot_pose.x,
                                                       robot_pose.y)
                    if curr_robot_goal < min_robot_goal:
                        min_robot_goal = curr_robot_goal
                    elif curr_robot_goal > min_robot_goal + self.relax_distance_threshold:
                        result.relaxation = True
                        result.robot_x = robot_pose.x
                        result.robot_y = robot_pose.y
                        result.goal_x = nav_goal.target_pose.pose.position.x
                        result.goal_y = nav_goal.target_pose.pose.position.y
                        self.nav_client.cancel_goal()
            
            self.result = result
            ##################################################

            

            self.sent_goal = True

            return py_trees.common.Status.RUNNING

        # http://docs.ros.org/en/lunar/api/actionlib_msgs/html/msg/GoalStatus.html
        # Checking the '/move_base' action client
        state = self.nav_client.get_state()
        if state == 2 or state == 4 or state == 5: # PREEMPTED, ABORTED, REJECTED
            print("Navigation cancelled")
            # if not self.result.relaxation:
            #     self.rviz_msg = None
            self.nav_status_pub.publish(self.result)
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                             (self.__class__.__name__, 
                              self.status, 
                              py_trees.common.Status.FAILURE, 
                              self.feedback_message))
            return py_trees.common.Status.FAILURE
        elif state == 3: # SUCCEEDED
            self.nav_status_pub.publish(self.result)
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                             (self.__class__.__name__, 
                              self.status, 
                              py_trees.common.Status.SUCCESS, 
                              self.feedback_message))

            # if self.blackboard.wm_msg['param_num'] == str(self.idx):
            #     self.rviz_msg = None
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.nav_client.get_state() == 1: # Active
            self.nav_client.cancel_goal()
        self.logger.debug("%s.terminate()[%s->%s]" % \
                          (self.__class__.__name__, 
                           self.status, 
                           new_status))

        return
    
    def robot_pose_callback(self, msg):
        with self.lock:
            self.robot_pose = msg.pose.pose.position
        
        # rospy.loginfo(f"Robot pose received: {self.robot_pose.x}, {self.robot_pose.y}")
    
    # Deprecated
    def nav_move_base(self, goal):
        self.nav_client.send_goal(goal)

        if self.robot_pose is None:
            rospy.loginfo("Robot pose not received yet!")
             
            # Wait for 1s for the self.robot_pose
            while self.robot_pose is None:
                rospy.loginfo("Waiting for robot pose...")
                rate = rospy.Rate(1)
                rate.sleep()
        
        if self.relaxation_set:
            min_robot_goal = self.l1_distance(goal.target_pose.pose.position.x, 
                                              goal.target_pose.pose.position.y,
                                              self.robot_pose.x, 
                                              self.robot_pose.y)

        result = TaskPlanResult()
        result.relaxation = False
        result.robot_x = 0
        result.robot_y = 0
        result.goal_x = 0
        result.goal_y = 0

        # Wait for 1s for the response from move_base
        while not self.nav_client.wait_for_result(rospy.Duration(1.0)):
            # if self.rviz_msg == "Cancel":
            #     self.nav_client.cancel_goal()
            #     rospy.loginfo(f"Navigation Plan Cancelled!")
            # elif self.rviz_msg == "Pause":
            #     self.nav_client.cancel_goal()
            #     rospy.loginfo(f"Navigation Plan Paused!")

            if self.relaxation_set:
                # Check relaxation trigger
                curr_robot_goal = self.l1_distance(goal.target_pose.pose.position.x,
                                                   goal.target_pose.pose.position.y,
                                                   self.robot_pose.x,
                                                   self.robot_pose.y)
                if curr_robot_goal < min_robot_goal:
                    min_robot_goal = curr_robot_goal
                elif curr_robot_goal > min_robot_goal + self.relax_distance_threshold:
                    result.relaxation = True
                    result.robot_x = self.robot_pose.x
                    result.robot_y = self.robot_pose.y
                    result.goal_x = goal.target_pose.pose.position.x
                    result.goal_y = goal.target_pose.pose.position.y
                    self.nav_client.cancel_goal()

        # Maybe, after the loop, we may check the state of self.nav_client to determine 
        # if the goal was achieved or if there was an error, and handle accordingly.
        # Will this be necessary?

        # if self.rviz_msg == "Pause":
        #     while self.rviz_msg == "Pause":
        #         rate = rospy.Rate(1)
        #         rate.sleep()
        #     if self.rviz_msg == "Cancel":
        #         rospy.loginfo(f"Navigation Plan Cancelled!!")
        #     elif self.rviz_msg == "Execute":
        #         rospy.loginfo(f"Navigation Plan Resumed!")
        #         result = self.nav_move_base(goal)

        return result
    
    @staticmethod
    # l2: ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    def l1_distance(x1, y1, x2, y2):
        return abs(x2 - x1) + abs(y2 - y1)