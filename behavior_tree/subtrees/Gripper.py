import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
from . import Move

from riro_srvs.srv import StringGoalStatus

class GOTO(Move.MOVE):
    """
    Move a gripper to the desired configuration

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """
    def __init__(self, name, action_client, action_goal=None, force=1., check_contact=False, timeout=5, robot_name=None):
        super(GOTO, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal,
                                   robot_name=robot_name,
                                   goal_channel="gripper")

        self.force         = force
        self.check_contact = check_contact
        self.timeout       = timeout


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)            
            cmd_str = json.dumps({'action_type': 'gripperGotoPos',
                                  'goal': self.action_goal,
                                  'uuid': self.goal_uuid_des.tolist(),
                                  'goal_channel': self.goal_channel,
                                  'force': self.force,
                                  'check_contact': self.check_contact,
                                  'timeout': self.timeout,
                                  'enable_wait': True})
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a gripper goal"
            self.debug_log(
                "gripper_command_requested",
                action_goal=self.action_goal,
                force=self.force,
                check_contact=self.check_contact,
                timeout=self.timeout,
            )
            return py_trees.common.Status.RUNNING

        current_goal_id = self.current_goal_id()
        current_goal_status = self.current_goal_status()

        if current_goal_id is None:
            self.debug_log_snapshot(
                "gripper_waiting_for_blackboard_goal",
                blackboard_goal_id=None,
                blackboard_goal_status=current_goal_status,
                blackboard_goal_status_name=self.goal_status_to_string(current_goal_status),
                goal_matches=False,
            )
            return py_trees.common.Status.RUNNING

        goal_matches = self.goal_matches_blackboard()
        self.debug_log_snapshot(
            "gripper_blackboard_state",
            blackboard_goal_id=current_goal_id,
            blackboard_goal_status=current_goal_status,
            blackboard_goal_status_name=self.goal_status_to_string(current_goal_status),
            goal_matches=goal_matches,
        )

        if goal_matches and \
           current_goal_status in [
                                GoalStatus.STATUS_UNKNOWN,
                                ]:
            self.feedback_message = "FAILURE"
            self.debug_log(
                "gripper_terminal_failure",
                blackboard_goal_id=current_goal_id,
                blackboard_goal_status=current_goal_status,
                blackboard_goal_status_name=self.goal_status_to_string(current_goal_status),
                goal_matches=goal_matches,
            )
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                  self.feedback_message))
            return py_trees.common.Status.FAILURE

        if goal_matches and \
           current_goal_status in [GoalStatus.STATUS_ABORTED,
                                   GoalStatus.STATUS_SUCCEEDED,
                                   GoalStatus.STATUS_CANCELING,
                                   GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "SUCCESSFUL"
            self.debug_log(
                "gripper_terminal_success",
                blackboard_goal_id=current_goal_id,
                blackboard_goal_status=current_goal_status,
                blackboard_goal_status_name=self.goal_status_to_string(current_goal_status),
                goal_matches=goal_matches,
            )
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
