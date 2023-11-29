import numpy as np
import json
import rospy

import py_trees
import std_msgs.msg as std_msgs
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryResult

## from complex_action_client.arm_client import ArmClient
from complex_action_client.srv import String_Int, None_String
from behavior_tree.subtrees import MoveJoint, MoveBase, WorldModel

class Idle(py_trees.composites.Sequence):
    """
    Move Pose
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, type, controller_ns='',):
        super(Idle, self).__init__(name=name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.drive_config = eval(rospy.get_param("drive_config", str([0, 0, 0, 0, 0, 0])))
        self.controller_ns  = controller_ns
        self.sent_goal   = False
        if type == 'manip':
            s_drive_pose = MoveJoint.MOVEJ(name="IdleInit", controller_ns=controller_ns, action_goal=self.blackboard.drive_config)
            pose_est10   = WorldModel.PARKING_POSE_ESTIMATOR(name='IdlePlan', object_dict = {'destination': 'home'})
            s_drive10    = MoveBase.MOVEB(name="IdleGoHome", action_goal={'pose': name+'Plan/home_pose'})
            idle         = py_trees.behaviours.Running(name="Idle")
            self.add_children([s_drive_pose, pose_est10, s_drive10, idle])
        elif type == 'quad':
            pose_est10   = WorldModel.PARKING_POSE_ESTIMATOR(name='IdlePlan', object_dict = {'destination': 'home'})
            s_drive10    = MoveBase.MOVEB(name="IdleGoHome", action_goal={'pose': name+'Plan/home_pose'})
            idle         = py_trees.behaviours.Running(name="Idle")
            self.add_children([pose_est10, s_drive10, idle])
        else:
            raise NotImplementedError
        
    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    # def update(self):
    #     self.logger.debug("%s.update()" % self.__class__.__name__)

    #     if not self.sent_goal:
    #         ## for req in self.cmd_req:
    #         ##     req( json.dumps({'action_type': 'cancel_goal'}) )
    #         self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
    #         self.sent_goal = True
    #         self.feedback_message = "Cancelling a goal"
    #     return py_trees.common.Status.SUCCESS

        
    def terminate(self, new_status):
        return
