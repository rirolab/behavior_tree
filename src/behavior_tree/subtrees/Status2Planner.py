#!/usr/bin/env python3

# standard imports
import re
import json
import threading

# ROS imports
import rospy
import actionlib
import py_trees
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus

# local imports
from riro_navigation.msg import TaskPlanAction, TaskPlanResult, TaskPlanResultTemp
from riro_navigation.srv import getRegionGoal
from riro_navigation.srv import NavigationControl, NavigationControlResponse


class TASKPLANCOMM(py_trees.behaviour.Behaviour):
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
    def __init__(self, name, idx='', action_goal=None, destination=None):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        super(TASKPLANCOMM, self).__init__(name=name)

        self.task_plan = None
        self.robot_pose = None
        self.rviz_msg = None

        # Mode
        self.relaxation_set = True


    # setup(self) handles all other one-time initialisations of resources 
    # that are required for execution:
    # Essentially, all the things that the constructor doesn’t handle
    # - hardware connections, middleware and other heavy resources.
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
        
        # ROS server, subscribed by product_automata_planner.py
        self.task_plan_server = actionlib.SimpleActionServer('task_plan', TaskPlanAction, 
                                                             self.execute_task_plan, False)
        self.task_plan_server.start()

        # This waits for MoveGoal.py(subtree) to send the navigation status
        # Maybe using ROS APIs would be more suitable, but it is not implemented yet
        # Initially set to False
        self.TaskPlanResult_temp_received = threading.Event()

        # ROS Subscribers
        rospy.Subscriber('/nav_result_temp', TaskPlanResultTemp, self.TaskPlanResult_temp_callback)

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

        self.TaskPlanResult_temp = None

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

        if self.TaskPlanResult_temp == None:
            return py_trees.common.Status.RUNNING

        # To be sent to product_automata_planner.py, asking for relaxation if necessary
        result = TaskPlanResult()

        if self.TaskPlanResult_temp.nav_client_state == 2: # PREEMPTED
            if not self.TaskPlanResult_temp.relaxation:
                self.rviz_msg = None
            
            result.relaxation = self.TaskPlanResult_temp.relaxation
            result.robot_x = self.TaskPlanResult_temp.robot_x
            result.robot_y = self.TaskPlanResult_temp.robot_y
            result.goal_x = self.TaskPlanResult_temp.goal_x
            result.goal_y = self.TaskPlanResult_temp.goal_y

            self.task_plan_server.set_preempted(result)
            return
        
        elif self.TaskPlanResult_temp.nav_client_state == 4: # ABORTED
            self.rviz_msg = None

            result.relaxation = self.TaskPlanResult_temp.relaxation
            result.robot_x = self.TaskPlanResult_temp.robot_x
            result.robot_y = self.TaskPlanResult_temp.robot_y
            result.goal_x = self.TaskPlanResult_temp.goal_x
            result.goal_y = self.TaskPlanResult_temp.goal_y

            self.task_plan_server.set_aborted(result)
            return
        
        self.task_plan_server.set_succeeded(result)
        rospy.loginfo(f"Navigation Plan Complete!")

        blackboard = py_trees.Blackboard()

        if blackboard.current_goal_idx == blackboard.goal_num:
            self.rviz_msg = None
            return py_trees.common.Status.SUCCESS
    
        blackboard.current_goal_idx += 1
        return py_trees.common.Status.SUCCESS

    def TaskPlanResult_temp_callback(self, msg):
        self.TaskPlanResult_temp = msg

    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        
        return