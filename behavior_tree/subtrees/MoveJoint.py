import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
from . import Move

from riro_srvs.srv import StringGoalStatus

class MOVEJ(Move.MOVE):
    """
    Move to the desired joint angles.

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """
    def __init__(self, name, action_client, action_goal=None, timeout=2.0,
                 robot_name=None, gripper=None, gripper_ramp_param=None,
                 claim=None, wait_until_done=False):
        """
        Args:
            gripper: a gripper position to carry through this motion's own
                trajectory, or None for no gripper intent at all. Not part of
                ``action_goal``: the goal is the chain the planner plans, and
                this is a column the planner does not own, so it travels beside
                it and cac appends it (``_append_passthrough_columns``). None
                means the column is OMITTED, which is how a motion says it has
                nothing to say about the jaws (rule i) -- not the same as
                sending their current position, which two live sources cannot
                tell from a deliberate "stay open".
            gripper_ramp_param: the node parameter holding the fraction of this
                motion by which a carried gripper must have arrived -- this
                motion's own out-seam. Read live, since the composite that
                chains this motion reads the same parameter live and the two
                must not overlap at two numbers.
            claim: this motion's own request fields -- see ``Move.MOVE``. A
                chain file's step states its seam here
                (``claim_progress_threshold``), the step after a policy adds
                ``plan_from_arm``, and a step whose file names a ramp states
                that too, which is why the claim is merged after the value
                ``gripper_ramp_param`` produces.
        """
        super(MOVEJ, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal,
                                   timeout=timeout,
                                   robot_name=robot_name,
                                   claim=claim, wait_until_done=wait_until_done)
        self.gripper = gripper
        self.gripper_ramp_param = gripper_ramp_param
        self.logger.debug("%s.__init__()" % self.__class__.__name__)

    def gripper_ramp_fraction(self):
        """The fraction of this motion by which a carried gripper must be there.

        This motion's out-seam. A column shaped over the whole motion has
        delivered only s(threshold) of its target when the seam opens, which is
        a gripper visibly not yet where the policy about to take over was
        trained to find it. Stated as a fraction rather than in seconds since
        the planned duration is trajectory_manager's answer, not the tree's.

        None with no node, no named parameter, or a value that is not a fraction
        -- and the ramp then defaults to the whole motion, which is what every
        motion did before the key existed and is the right answer at threshold
        1.0 anyway.
        """
        if self._node is None or self.gripper_ramp_param is None:
            return None
        try:
            value = self._node.get_parameter(self.gripper_ramp_param).value
        except Exception:
            return None
        try:
            fraction = float(value)
        except (TypeError, ValueError):
            return None
        if not 0.0 < fraction <= 1.0:
            return None
        return fraction

    def make_command(self, uuid=None, enable_wait=False):
        """
        Export this joint move as a complex action client command dictionary.
        """
        # Encode joint targets with the same indexed schema used by update().
        goal = {}
        for i, ang in enumerate(self.action_goal):
            goal[str(i)] = ang
        carried = {}
        if self.gripper is not None:
            carried["gripper"] = float(self.gripper)
            fraction = self.gripper_ramp_fraction()
            if fraction is not None:
                carried["passthrough_ramp_fraction"] = fraction
        return self._make_command(
            "moveJoint", json.dumps(goal), uuid=uuid, enable_wait=enable_wait,
            **carried
        )

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.common.Status.FAILURE
            
        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)
            cmd_str = json.dumps(
                self.make_command(uuid=self.goal_uuid_des.tolist(), enable_wait=False)
            )
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        self.feedback_message = "running"

        # Handle complex action client rejection before waiting for a goal-status topic.
        command_status = self.command_response_status()
        if command_status is not None:
            return command_status

        if self.current_goal_id() is None:
            return self.still_waiting()
        
        if self.goal_matches_blackboard() and \
           self.current_goal_status() in [GoalStatus.STATUS_ABORTED,
                                GoalStatus.STATUS_UNKNOWN,
                                GoalStatus.STATUS_CANCELING,
                                GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                  self.feedback_message))
            return py_trees.common.Status.FAILURE

        if self.goal_matches_blackboard() and \
           self.current_goal_status() == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return self.still_waiting()
                



class MOVEJR(Move.MOVE):
    """
    Move a desired displacement of joint angles.

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None, robot_name=None):
        # Match the relative-joint command timeout used by update().
        super(MOVEJR, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal,
                                   timeout=3.,
                                   robot_name=robot_name)

    def make_command(self, uuid=None, enable_wait=False):
        """
        Export this relative joint move as a complex action client command dictionary.
        """
        # Encode relative joint targets with the same indexed schema used by update().
        goal = {}
        for i, ang in enumerate(self.action_goal):
            goal[str(i)] = ang
        return self._make_command(
            "moveJointRelative", json.dumps(goal), uuid=uuid, enable_wait=enable_wait,
        )

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
            
        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)
            cmd_str = json.dumps(
                self.make_command( uuid=self.goal_uuid_des.tolist(), enable_wait=False)
            )
            req = StringGoalStatus.Request(data=cmd_str)            
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        # Handle complex action client rejection before waiting for a goal-status topic.
        command_status = self.command_response_status()
        if command_status is not None:
            return command_status

        if self.current_goal_id() is None:
            return py_trees.common.Status.RUNNING
            
        if self.goal_matches_blackboard() and \
           self.current_goal_status() in [GoalStatus.STATUS_ABORTED,
                                GoalStatus.STATUS_UNKNOWN,
                                GoalStatus.STATUS_CANCELING,
                                GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                  self.feedback_message))
            return py_trees.common.Status.FAILURE

        if self.goal_matches_blackboard() and \
           self.current_goal_status() == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
                
