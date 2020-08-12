import numpy as np
import json
import rospy

import py_trees
import std_msgs.msg as std_msgs
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryResult

## from complex_action_client.arm_client import ArmClient
from complex_action_client.srv import String_Int, None_String

class STOP(py_trees.behaviour.Behaviour):
    """
    Move Pose
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None,
                 topic_name="", controller_ns=""):
        super(STOP, self).__init__(name=name)

        self.topic_name = topic_name
        self.controller_ns = controller_ns
        self.arm = None
        self.action_goal = action_goal
        self.sent_goal = False
        self.cmd_req   = None


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        ## self.arm = ArmClient(timeout_scale=1., controller_ns=self.controller_ns,
        ##                      use_gripper=False)
        rospy.wait_for_service("arm_client/command")
        self.cmd_req = rospy.ServiceProxy("arm_client/command", String_Int)
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
            ## self.arm._client.cancel_goal()
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
            self.sent_goal = True
            self.feedback_message = "Cancelling a goal"
        return py_trees.common.Status.SUCCESS

        ## state = self.arm.get_state()
        ## ret   = self.arm.get_result()
        
        ## if  state in [GoalStatus.ABORTED,
        ##               GoalStatus.PREEMPTED] and \
        ##               ret != FollowJointTrajectoryResult.SUCCESSFUL: 
        ##     self.feedback_message = "SUCCESSFUL"
        ##     return py_trees.common.Status.SUCCESS

        ## if ret == FollowJointTrajectoryResult.SUCCESSFUL:
        ##     self.feedback_message = "SUCCESSFUL"
        ##     return py_trees.common.Status.SUCCESS
        ## else:
        ##     self.feedback_message = "RUNNING"
        ##     return py_trees.common.Status.RUNNING

        
    def terminate(self, new_status):
        ## self.arm._client.cancel_goal()
        return
