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

    The one gripper behaviour, on every wiring. Which writer carries the
    position out -- a gripper action server of its own, or the arm controller's
    column that the mixer passes through -- is resolved by
    ``ArmClient._gripper_writer`` and nowhere else, so this command names no
    channel and this behaviour reads whichever channel reports its goal id. A
    second behaviour per wiring, which is what ``GOTO_VIA_ARM`` was, makes the
    choice of subtree a deployment decision taken in the tree.

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
                                   goal_channel=None)

        self.force         = force
        self.check_contact = check_contact
        self.timeout       = timeout

        # A gripper's own motion is its own step, never overlapped *into*.
        #
        # Read by MoveBlend of the NEXT child (`MoveBlend.py:375`), so this
        # forbids starting THIS step early -- closing before the arm has
        # reached the grasp, opening before it has reached the place. It does
        # not forbid the seam out of it: that one is decided by this step's
        # `current_progress()` against the next step's threshold, and held by
        # the mixer until the jaws are still.
        #
        # Explicit, since `Move.py` derives the default from the stream slots
        # and this behaviour has them: on a deployment whose gripper is a
        # passthrough column the motion really does go out through a slot.
        self.overlappable = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)            
            cmd_str = json.dumps({'action_type': 'gripperGotoPos',
                                  'goal': self.action_goal,
                                  'uuid': self.goal_uuid_des.tolist(),
                                  'force': self.force,
                                  'check_contact': self.check_contact,
                                  'timeout': self.timeout,
                                  'enable_wait': True})
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a gripper goal"
            return py_trees.common.Status.RUNNING

        # Handle complex action client rejection before waiting for a goal-status topic.
        command_status = self.command_response_status()
        if command_status is not None:
            return command_status

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
