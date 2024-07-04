#!/usr/bin/env python3

# standard imports
import re
import json

# ROS imports
import rospy
import actionlib
import py_trees
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry

# local imports
from riro_navigation.msg import TaskPlanAction, TaskPlanResult
from riro_navigation.srv import getRegionGoal
from riro_navigation.srv import NavigationControl, NavigationControlResponse
from std_msgs.msg import String

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

        # parameter settings
        if sim == 'true' or sim == 'True':
            self.sim = True
            self.map_frame = 'map_carla'
            self.relax_distance_threshold = 10
            rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.robot_pose_callback)
        elif sim == 'false' or sim == 'False':
            self.sim = False
            self.map_frame = 'map'
            self.relax_distance_threshold = 3
            rospy.Subscriber("/odom", Odometry, self.robot_pose_callback)
        else:
            self.sim = False
            print("sim arg strange")

        self.task_plan = None
        self.robot_pose = None
        self.rviz_msg = None

        # Mode
        self.relaxation_set = True


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

        if self.sim:
            rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.robot_pose_callback)
        elif not self.sim:
            rospy.Subscriber("/odom", Odometry, self.robot_pose_callback)
        else:
            print("sim arg strange")
        
        # ROS server, subscribed by product_automata_planner.py
        self.task_plan_server = actionlib.SimpleActionServer('task_plan', TaskPlanAction, 
                                                             self.execute_task_plan, False)
        self.task_plan_server.start()

        # ROS client
        self.nav_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.getregiongoal_client = rospy.ServiceProxy('/manage_map/get_region_goal', getRegionGoal)

        # ROS publihser
        # self.status_pub publishes the navigation status to product_automata_planner.py
        self.status_pub = rospy.Publisher('/move_goal_status', String, queue_size=10)

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

        blackboard = py_trees.Blackboard()

        self.task_plan = None
        self.robot_pose = None
        self.rviz_msg = None

        # Mode
        self.relaxation_set = True

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

        blackboard = py_trees.Blackboard()

        goal_name = self.action_goal['pose'] # ex) r1, r3, etc.
        is_last_goal, goal_loc = self.get_goal_info_from_wm(goal_name)
        
        # goal to send to move_base
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        # When the task_plan is format of 'r1,' 'r5,' etc.
        if re.match(r'r\d+', goal_name):
            # Get the goal from service '/manage_map/get_region_goal'
            rospy.wait_for_service('/manage_map/get_region_goal') # In line 369 of manage_map.cpp
            response = self.getregiongoal_client(int(goal_name[1:]))

            goal.target_pose.pose.position.x = response.goal_x
            goal.target_pose.pose.position.y = response.goal_y
        else:
            goal.target_pose.pose.position.x = goal_loc.goal_x
            goal.target_pose.pose.position.y = goal_loc.goal_y
        goal.target_pose.pose.position.z = 0

        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = 0
        goal.target_pose.pose.orientation.w = 1

        if self.sim == False:
            rate = rospy.Rate(1)
            rospy.loginfo(f"Navigation Plan Ready!")
            while self.rviz_msg == None:
                rate.sleep()
        rospy.loginfo(f"Navigation Plan Being Executed!")

        result = self.nav_move_base(goal)

        # http://docs.ros.org/en/lunar/api/actionlib_msgs/html/msg/GoalStatus.html
        # Checking the '/move_base' action client
        if self.nav_client.get_state() == 2: # PREEMPTED
            print("Navigation cancelled")
            if not result.relaxation:
                self.rviz_msg = None
            # self.task_plan_server.set_preempted(result)
            self.status_pub.publish("Cancelled")
            return
        elif self.nav_client.get_state() == 4: # ABORTED
            print("Navigation aborted")
            self.rviz_msg = None
            # self.task_plan_server.set_aborted(result)
            self.status_pub.publish("Aborted")
            return

        # print("Navigation succeed")
        # self.task_plan_server.set_succeeded(result)
        self.status_pub.publish("Succeeded")
        rospy.loginfo(f"Navigation Plan Complete!")
        if is_last_goal:
            self.rviz_msg = None
        return


    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        
        blackboard = py_trees.Blackboard()
        # self.drive_status_update_req(blackboard.robot_name)
        return
    
        # Returns the goal location from the world model
    
    def get_goal_info_from_wm(self, goal_name):
        blackboard = py_trees.Blackboard()
        wm_msg = json.loads(blackboard.wm_msg.data)["world"]

        goal_location = None

        for wm_obj in wm_msg:
            wm_obj_name = wm_obj["name"]
            if wm_obj_name == goal_name:
                goal_location = wm_obj["location"]
                is_final_goal = wm_obj["final"] #True, False
        
        return is_final_goal, goal_location
    
    # Not used in the current implementation, just for legacy purposes
    def robot_pose_callback(self, msg):
        self.robot_pose = msg.pose.pose.position
    
    def nav_move_base(self, goal):
        self.nav_client.send_goal(goal)
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
            # Not used
            if self.rviz_msg == "Cancel":
                self.nav_client.cancel_goal()
                rospy.loginfo(f"Navigation Plan Cancelled!")
            elif self.rviz_msg == "Pause":
                self.nav_client.cancel_goal()
                rospy.loginfo(f"Navigation Plan Paused!")

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

        return result
    
    @staticmethod
    # l2: ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    def l1_distance(x1, y1, x2, y2):
        return abs(x2 - x1) + abs(y2 - y1)