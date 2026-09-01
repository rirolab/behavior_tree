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
            return py_trees.common.Status.RUNNING

        if self.current_goal_id() is None:
            return py_trees.common.Status.RUNNING
            
        if self.goal_matches_blackboard() and \
           self.current_goal_status() in [
                                GoalStatus.STATUS_UNKNOWN,
                                ]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                  self.feedback_message))
            return py_trees.common.Status.FAILURE

        if self.goal_matches_blackboard() and \
           self.current_goal_status() in [GoalStatus.STATUS_ABORTED,
                                               GoalStatus.STATUS_SUCCEEDED,
                                               GoalStatus.STATUS_CANCELING,
                                               GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING


class GOTO_VIA_ARM(Move.MOVE):
    """
    Open/close the gripper by sending an 8-joint FJT to the arm controller.

    For ffw_bg2 real robot where the gripper joint is part of the arm
    controller's JTC (no dedicated gripper action server). The arm holds
    its current desired angles and only the gripper joint is commanded.

    The goal channel is ``arm``, not ``gripper``, even though this is a gripper
    op: ``gripperGotoPosViaArm`` takes ``**kwargs`` and never forwards a channel
    to ``_send_goal``, which defaults to ``arm``, so that is where the status
    actually appears. The command dispatcher would classify it as ``gripper``
    from its name, and reading the gripper channel here would wait forever.
    """
    def __init__(self, name, action_client, action_goal=None, timeout=3.0, robot_name=None):
        super(GOTO_VIA_ARM, self).__init__(name=name,
                                           action_client=action_client,
                                           action_goal=action_goal,
                                           robot_name=robot_name,
                                           goal_channel="arm")
        self.timeout = timeout
        # Never start the next motion early over this one, and never start this
        # one early. It reaches the arm controller as a whole FJT goal, not
        # through a mixer slot, so nothing blends it: dispatched early it would
        # act at once -- closing before the arm has reached the grasp, opening
        # before it has reached the place. `overlappable` would otherwise be
        # True, inherited from the arm channel's stream slots.
        self.overlappable = False

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16, dtype=np.uint8)
            cmd_str = json.dumps({
                "action_type": "gripperGotoPosViaArm",
                "goal": self.action_goal,
                "uuid": self.goal_uuid_des.tolist(),
                "timeout": self.timeout,
                "enable_wait": False,
            })
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            self.sent_goal = True
            self.feedback_message = "Sending a gripper-via-arm goal"
            return py_trees.common.Status.RUNNING

        if self.current_goal_id() is None:
            return py_trees.common.Status.RUNNING
        if not self.goal_matches_blackboard():
            return py_trees.common.Status.RUNNING

        status = self.current_goal_status()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            return py_trees.common.Status.SUCCESS
        if status in [GoalStatus.STATUS_ABORTED,
                      GoalStatus.STATUS_CANCELING,
                      GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "FAILURE"
            return py_trees.common.Status.FAILURE

        self.feedback_message = "running"
        return py_trees.common.Status.RUNNING
