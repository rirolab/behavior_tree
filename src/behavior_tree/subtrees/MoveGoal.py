#!/usr/bin/env python3

# standard imports
import re
import json
import datetime

# ROS imports
import rospy
import time
import threading
import actionlib
import py_trees
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

# local imports
from riro_navigation.msg import TaskPlanResult
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
    def __init__(self, name, idx='', action_goal=None, destination=None, sim='true', waypoint_seq=None):
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
        self.min_dist = 10000000
        self.thresh = 10

        self.robot_pose = None

        # May be deprecated
        # self.rviz_msg = None

        # Mode
        self.relaxation_mode = True
        self.lock = threading.Lock()
        self.result = None
        self.sent_goal = False
        self.sent_goal_to_planner = False
        self.waypoint_seq = waypoint_seq


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

        sim = rospy.get_param('sim', False)
        
        self.sim = sim
        if self.sim:
            self.map_frame = 'map_carla' 
            self.relax_distance_threshold = 10
            rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.robot_pose_callback)
        else:
            self.map_frame = 'custom_costmap'
            self.relax_distance_threshold = 3
            rospy.Subscriber("/odom_spot", Odometry, self.robot_pose_callback)

        rospy.Subscriber("/planner_ready", String, self.planner_ready_callback)
        rospy.Subscriber("/move_base/GlobalPlanner/plan", Path, self.global_plan_callback)

        # ROS client
        self.nav_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        server_up = self.nav_client.wait_for_server(timeout=rospy.Duration.from_sec(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for MoveBase"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
        self.getregiongoal_client = rospy.ServiceProxy('/manage_loaded_map/get_region_goal', getRegionGoal)

        # ROS publihser
        self.nav_status_pub = rospy.Publisher('/status_to_planner', TaskPlanResult, queue_size=1)
        self.planner_goal_pub = rospy.Publisher('/planner_goal', String, queue_size=1)
        self.planner_result_pub = rospy.Publisher("/planner_result", String, queue_size=10)
        self.cancel_pub = rospy.Publisher("symbol_grounding", String, queue_size=10)
        self.nav_goal = None
        
        # added to ensure safe publishing
        while self.planner_goal_pub.get_num_connections() < 1 or \
            self.cancel_pub.get_num_connections() < 1 or \
            self.planner_result_pub.get_num_connections() < 1:
            rospy.sleep(0.1)
        
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
        self.planner_ready = False
        self.sent_goal = False
        self.sent_goal_to_planner = False

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
        if not self.sent_goal_to_planner:

            # Setting the goal
            self.nav_goal = MoveBaseGoal()

            # Setting MoveBaseGoal()'s header(std_msgs/Header)
            rospy.loginfo(f".............Setting MoveBaseGoal() map frame: {self.map_frame}")
            self.nav_goal.target_pose.header.frame_id = self.map_frame
            self.nav_goal.target_pose.header.stamp = rospy.Time.now()

            # When the task_plan is format of 'r1,' 'r5,' etc.
            if re.match(r'r\d+', self.destination):
                # Get the goal from service '/manage_loaded_map/get_region_goal'
                rospy.wait_for_service('/manage_loaded_map/get_region_goal')
                response = self.getregiongoal_client(int(self.destination[1:]))

                self.nav_goal.target_pose.pose.position.x = response.goal_x
                self.nav_goal.target_pose.pose.position.y = response.goal_y
            else:
                self.nav_goal.target_pose.pose.position.x = np.float64(self.action_goal['pose']['x'])
                self.nav_goal.target_pose.pose.position.y = np.float64(self.action_goal['pose']['y'])
            self.nav_goal.target_pose.pose.position.z = np.float64(0)
            
            self.nav_goal.target_pose.pose.orientation.x = np.float64(0)
            self.nav_goal.target_pose.pose.orientation.y = np.float64(0)
            self.nav_goal.target_pose.pose.orientation.z = np.float64(0)
            self.nav_goal.target_pose.pose.orientation.w = np.float64(1)
            
            rospy.loginfo(f"(moveGoal) goal! {self.nav_goal}")

            ######### Existing nav_move_base() code #########
           
            # TODO : send goals to product_automata_planner.py via ROS topic
            waypoint_infos = {}
            for wp in self.waypoint_seq:
                for _, v in self.blackboard.wm_dict.items():
                    if v['name'] == wp:
                        waypoint_infos[wp] = {'location' : v['location'],
                                              'final' : v['final'],
                                              'relax_point' : v['relax_point']}
            planner_msg = {
                "waypoint_seq" : self.waypoint_seq,
                "waypoint_infos" : waypoint_infos
            }
            planner_msg = json.dumps(planner_msg)
            self.planner_goal_pub.publish(planner_msg)
            rospy.logerr(f"Sent goals to planner: {planner_msg}")
            
            # self.nav_client.send_goal(nav_goal)
            self.sent_goal_to_planner = True
            return py_trees.common.Status.RUNNING
        
        elif not self.planner_ready:
            return py_trees.common.Status.RUNNING
            
        elif not self.sent_goal:
            self.nav_client.send_goal(self.nav_goal)
            self.sent_goal = True
            return py_trees.common.Status.RUNNING

        # http://docs.ros.org/en/lunar/api/actionlib_msgs/html/msg/GoalStatus.html
        # Checking the '/move_base' action client
        state = self.nav_client.get_state()
        rospy.logerr(f"Navigation state: {state}")
        if state == 3: # SUCCEEDED (goal reached)
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                             (self.__class__.__name__, 
                              self.status, 
                              py_trees.common.Status.SUCCESS, 
                              self.feedback_message))
            status_dict = {'status' : 'success'}
            self.planner_result_pub.publish(json.dumps(status_dict))
            return py_trees.common.Status.SUCCESS

        # elif state == 2 or state == 4 or state == 5: # PREEMPTED, ABORTED, REJECTED
        elif state == 2 or state == 5: # PREEMPTED, REJECTED
            # TODO? : should we handle when the goal is preempted / rejected?
            print("Navigation cancelled")
            self.nav_status_pub.publish(self.result)
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                             (self.__class__.__name__, 
                              self.status, 
                              py_trees.common.Status.FAILURE, 
                              self.feedback_message))
            status_dict = {'status' : 'failure'}
            self.planner_result_pub.publish(json.dumps(status_dict))
            return py_trees.common.Status.FAILURE
        elif state == 4:
            # TODO? : should we handle when the goal is aborted?
            # status_dict = {'status' : 'failure'}
            timestamp_now = str(datetime.datetime.now())
            ####### 1. Send stop command to product_automata_planner.py via ROS topic #######
            taskplangoals_dict = {1: {
                                        "primitive_action": "stop",
                                    },
                                }

            # Create final dictionary 'd' that is to be published to Black Board
            d = {
                "timestamp": timestamp_now,
                "params": taskplangoals_dict,
                "param_num": 1
            }
            self.cancel_pub.publish(json.dumps(d))
            status_dict = {'status' : 'failure',
                              'robot_location' : {'x' : self.robot_pose.x, 'y' : self.robot_pose.y},
                              'location' : {'x' : float(self.action_goal['pose']['x']), 'y' : float(self.action_goal['pose']['y'])}}
            self.planner_result_pub.publish(json.dumps(status_dict))
            return py_trees.common.Status.SUCCESS
        
        elif state == 1 and self.relaxation_mode:
            # curr_dist = self.l1_distance(self.robot_pose.x,self.robot_pose.y,
            #                             float(self.action_goal['pose']['x']), float(self.action_goal['pose']['y']))
            curr_dist = self.compute_path_length(self.global_plan)
            if curr_dist < self.min_dist:
                self.min_dist = curr_dist
                return py_trees.common.Status.RUNNING
            elif curr_dist > self.min_dist + self.thresh:
                timestamp_now = str(datetime.datetime.now())
                ####### 1. Send stop command to product_automata_planner.py via ROS topic #######
                taskplangoals_dict = {1: {
                                            "primitive_action": "stop",
                                        },
                                    }

                # Create final dictionary 'd' that is to be published to Black Board
                d = {
                    "timestamp": timestamp_now,
                    "params": taskplangoals_dict,
                    "param_num": 1
                }
                self.cancel_pub.publish(json.dumps(d))
                
                ####### 2. Send relaxation trigger to product_automata_planner.py via ROS topic #######
                status_dict = {'status' : 'relax',
                              'robot_location' : {'x' : self.robot_pose.x, 'y' : self.robot_pose.y},
                              'location' : {'x' : float(self.action_goal['pose']['x']), 'y' : float(self.action_goal['pose']['y'])}}
                self.planner_result_pub.publish(json.dumps(status_dict))

                rospy.logerr("Canceled the goal!!!!!!!")
                return py_trees.common.Status.FAILURE
            else:
                return py_trees.common.Status.RUNNING
                

        else:
            with self.lock:
                robot_pose = self.robot_pose
            if robot_pose is None:
                return py_trees.common.Status.RUNNING

            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        rospy.logerr("Terminating moveGoal !!!!!!!!!!!!!!!!!!!!!!")
        if self.nav_client.get_state() == 1: # Active
            self.nav_client.cancel_goal()
        self.logger.debug("%s.terminate()[%s->%s]" % \
                          (self.__class__.__name__, 
                           self.status, 
                           new_status))

        return
    
    def compute_path_length(self, path):
        positions = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in path.poses])
        if positions.shape[0] < 2:
            return 0.0
        deltas = positions[1:] - positions[:-1]
        segment_lengths = np.hypot(deltas[:, 0], deltas[:, 1])
        total_length = np.sum(segment_lengths)
        return total_length
    
    def robot_pose_callback(self, msg):
        with self.lock:
            self.robot_pose = msg.pose.pose.position
        
        # rospy.loginfo(f"Robot pose received: {self.robot_pose.x}, {self.robot_pose.y}")
        
    def planner_ready_callback(self, msg):
        with self.lock:
            self.planner_ready = True
            
    def global_plan_callback(self, msg):
        with self.lock:
            self.global_plan = msg
        
    @staticmethod
    def l1_distance(x1, y1, x2, y2):
    # l2: ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        return abs(x2 - x1) + abs(y2 - y1)













########### tried to discriminate between relax and move, but it didn't work ############
    
# class RELAX(py_trees.behaviour.Behaviour):
#     """
#     Move Base
    
#     Note that this behaviour will return with
#     :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
#     command to the robot if it is cancelled or interrupted by a higher
#     priority behaviour.
#     """

#     # __init__(self) should instantiate the behaviour sufficiently for offline dot graph generation
#     # No hardware connections that may not be there, e.g. usb lidars
#     # No middleware connections to other software that may not be there, e.g. ROS pubs/subs/services
#     # No need to fire up other needlessly heavy resources, e.g. heavy threads in the background
#     def __init__(self, name, idx='', action_goal=None, destination=None, sim='true', waypoint_seq=None):
#         """
#         Minimal one-time initialisation. A good rule of thumb is
#         to only include the initialisation relevant for being able
#         to insert this behaviour in a tree for offline rendering to
#         dot graphs.

#         Other one-time initialisation requirements should be met via
#         the setup() method.
#         """
#         super(RELAX, self).__init__(name=name)

#         self.idx = idx
#         self.action_goal = action_goal # pose
#         self.destination = destination  # 'r1', 'r2', etc.
#         self.sim = sim
#         self.min_dist = 10000000
#         self.thresh = 10

#         self.robot_pose = None

#         # May be deprecated
#         # self.rviz_msg = None

#         # Mode
#         self.relaxation_mode = True
#         self.lock = threading.Lock()
#         self.result = None
#         self.sent_goal = False
#         self.waypoint_seq = waypoint_seq


#     # setup(self) handles all other one-time initialisations of resources that are required for execution:
#     # Essentially, all the things that the constructor doesn’t handle - hardware connections, middleware and other heavy resources.
#     def setup(self, timeout):
#         """
#         When is this called?
#           This function should be either manually called by your program
#           to setup this behaviour alone, or more commonly, via
#           :meth:`~py_trees.behaviour.Behaviour.setup_with_descendants`
#           or :meth:`~py_trees.trees.BehaviourTree.setup`, both of which
#           will iterate over this behaviour, it's children (it's children's
#           children ...) calling :meth:`~py_trees.behaviour.Behaviour.setup`
#           on each in turn.

#           If you have vital initialisation necessary to the success
#           execution of your behaviour, put a guard in your
#           :meth:`~py_trees.behaviour.Behaviour.initialise` method
#           to protect against entry without having been setup.

#         What to do here?
#           Delayed one-time initialisation that would otherwise interfere
#           with offline rendering of this behaviour in a tree to dot graph
#           or validation of the behaviour's configuration.

#           Good examples include:

#           - Hardware or driver initialisation
#           - Middleware initialisation (e.g. ROS pubs/subs/services)
#           - A parallel checking for a valid policy configuration after
#             children have been added or removed
#         """
#         # rospy.loginfo('[subtree] movebase: setup() called.')
#         # self.feedback_message = "{}: setup".format(self.name)

#         self.blackboard = py_trees.blackboard.Blackboard()

#         # parameter settings
#         # if self.sim == 'true' or self.sim == 'True':
#         #     self.sim = True
#         #     self.map_frame = 'map_carla' 
#         #     self.relax_distance_threshold = 10
#         #     rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.robot_pose_callback)
#         # elif self.sim == 'false' or self.sim == 'False':
#         #     self.sim = False
#         #     self.map_frame = 'map'
#         #     self.relax_distance_threshold = 3
#         #     rospy.Subscriber("/odom", Odometry, self.robot_pose_callback)
#         # else:
#         #     self.sim = False
#         #     print("sim arg strange")
#         # self.sim = True
#         # self.map_frame = 'map_carla' 
#         # self.relax_distance_threshold = 10
#         # rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.robot_pose_callback)

#         # ROS subscriber / publisher
#         rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, self.robot_pose_callback)
        
#         self.nav_status_pub = rospy.Publisher('/status_to_planner', TaskPlanResult, queue_size=10)
#         self.cancel_pub = rospy.Publisher("symbol_grounding", String, queue_size=10)
#         self.relaxation_pub = rospy.Publisher("relaxation_trigger", String, queue_size=10)
        
#         return True

#     # initialise(self) configures and resets the behaviour ready for (repeated) execution
#     def initialise(self):
#         """
#         When is this called?
#           The first time your behaviour is ticked and anytime the
#           status is not RUNNING thereafter.

#         What to do here?
#           Any initialisation you need before putting your behaviour
#           to work.
#         """
#         # rospy.loginfo('[subtree] movebase: initialise() called.')
#         # self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
#         # rospy.loginfo(f"{self.__class__.__name__}.intialise() called")

#         # self.sent_goal = False

#     def update(self):
#         """
#         When is this called?
#           Every time your behaviour is ticked.

#         What to do here?
#           - Triggering, checking, monitoring. Anything...but do not block!
#           - Set a feedback message
#           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
#         """
#         if not self.relaxation_mode:
#             return py_trees.common.Status.SUCCESS
        
#         curr_dist = self.l1_distance(self.robot_pose.x,self.robot_pose.y,
#                                      float(self.action_goal['pose']['x']), float(self.action_goal['pose']['y']))
#         if curr_dist < self.min_dist:
#             self.min_dist = curr_dist
#         elif curr_dist > self.min_dist + self.thresh:
#             rospy.logerr("SHOULD TRIGGER RELAXATION!!!!!!!!")
#             # call relaxation to product_automata_planner.py
            
            
#             # call stop_cmd to cancel out all tasks
#             timestamp_now = str(datetime.datetime.now())
#             taskplangoals_dict = {1: {
#                                         "primitive_action": "stop",
#                                     },
#                                 }

#             # Create final dictionary 'd' that is to be published to Black Board
#             d = {
#                 "timestamp": timestamp_now,
#                 "params": taskplangoals_dict,
#                 "param_num": 1
#             }
#             self.cancel_pub.publish(json.dumps(d))
#             # self.relaxation_pub.publish("True")
#             # self.blackboard.set('stop_cmd', True)
#             rospy.logerr("Canceled the goal!!!!!!!")
            
#         # if robot reaches the goal
#         # then return py_trees.common.Status.SUCCESS
#         # else
#         # return py_trees.common.Status.RUNNING
        
#         return py_trees.common.Status.RUNNING

#     def terminate(self, new_status):
#         # if self.nav_client.get_state() == 1: # Active
#         #     self.nav_client.cancel_goal()
#         # self.logger.debug("%s.terminate()[%s->%s]" % \
#         #                   (self.__class__.__name__, 
#         #                    self.status, 0
#         #                    new_status))

#         return
    
#     def robot_pose_callback(self, msg):
#         with self.lock:
#             self.robot_pose = msg.pose.pose.position
    
    
#     @staticmethod
#     def l1_distance(x1, y1, x2, y2):
#     # l2: ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
#         return abs(x2 - x1) + abs(y2 - y1)

