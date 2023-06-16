import numpy as np
import json
import rospy

import py_trees
import std_msgs.msg as std_msgs
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryResult

from complex_action_client.srv import String_Int, None_String

class GOTO(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None, force=1., controller_ns="",
                 topic_name="", check_contact=False):
        super(GOTO, self).__init__(name=name)

        self.topic_name    = topic_name
        self.controller_ns = controller_ns
        self.action_goal   = action_goal
        self.force         = force
        self.sent_goal     = False
        self.cmd_req       = None
        self.check_contact = check_contact


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("arm_client/command")
        self.cmd_req = rospy.ServiceProxy("arm_client/command", String_Int)
        rospy.wait_for_service("arm_client/status")
        self.status_req = rospy.ServiceProxy("arm_client/status", None_String)        
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            cmd_str = json.dumps({'action_type': 'gripperGotoPos',
                                  'goal': self.action_goal,
                                  'force': self.force,
                                  'check_contact': self.check_contact,
                                  'timeout': 5})
            self.cmd_req(cmd_str)
            rospy.sleep(1)
            self.sent_goal = True
            self.feedback_message = "Sending a gripper goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d   = json.loads(msg.data)
        ret = d['gripper_state']

        ## state = self.arm.getGripperState()
        ## if  state in [GoalStatus.ABORTED,
        ##               GoalStatus.PREEMPTED] and \
        ##               ret != FollowJointTrajectoryResult.SUCCESSFUL: 
        ##     self.feedback_message = "FAILURE"
        ##     return py_trees.common.Status.FAILURE

        if ret == GoalStatus.SUCCEEDED or ret == GoalStatus.PREEMPTED :
            self.feedback_message = "SUCCESSFUL"
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
            
    
    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['gripper_state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )        
        return
