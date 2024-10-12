#!/usr/bin/env python3

# Standard imports
import json

# ROS imports
import rospy
import py_trees
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

class TASKPLANCOMM(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    # __init__(self) should instantiate the behaviour sufficiently for offline dot graph generation
    # No hardware connections that may not be there, e.g. usb lidars
    # No middleware connections to other software that may not be there, e.g. ROS pubs/subs/services
    # No need to fire up other needlessly heavy resources, e.g. heavy threads in the background
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        super(TASKPLANCOMM, self).__init__(name=name)

        # Variable to keep track of whether we have already notified the planner
        self.notified_planner = False

        # Store the latest status of move_base
        self.move_base_status = None

        # Blackboard variables
        self.blackboard = py_trees.blackboard.Blackboard()
    
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
        rospy.loginfo('[Status2Planner] setup() called.')

        # Subscribe to move_base status
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)

        # Publisher to send status to planner
        self.status_pub = rospy.Publisher('/status_to_planner', String, queue_size=10)

        return True

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        rospy.loginfo('[Status2Planner] initialise() called.')
        self.notified_planner = False

    def move_base_status_callback(self, msg):
        """
        Callback function to update the move_base status.
        """
        if msg.status_list:
            self.move_base_status = msg.status_list[-1]
        else:
            self.move_base_status = None

    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        rospy.loginfo('[Status2Planner] update() called.')

        # Check if we have already notified the planner
        if self.notified_planner:
            return py_trees.common.Status.SUCCESS

        # Check move_base status
        if self.move_base_status:
            status = self.move_base_status.status
            # If status is ABORTED (4) or REJECTED (5) or PREEMPTED (2), navigation failed
            if status in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED]:
                # Access the blackboard to get the current goal index
                current_goal_idx = self.blackboard.get('current_goal_idx', 1)
                rospy.loginfo(f"Navigation failed at goal index {current_goal_idx}, status: {status}")
                # Notify the planner
                data = {
                    'failed_goal_id': current_goal_idx,
                    'replan_requested': True
                }
                self.status_pub.publish(json.dumps(data))
                self.notified_planner = True
                return py_trees.common.Status.FAILURE
            elif status == GoalStatus.SUCCEEDED:
                # Navigation succeeded
                rospy.loginfo("Navigation succeeded.")
                return py_trees.common.Status.SUCCESS
            else:
                # Navigation is ongoing
                return py_trees.common.Status.RUNNING
        else:
            # No status received yet
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        Called when the behavior transitions to a new status.
        """
        rospy.loginfo('[Status2Planner] terminate() called.')
        # Clean up if necessary
