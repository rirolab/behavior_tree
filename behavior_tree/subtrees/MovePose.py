import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
import geometry_msgs
from behavior_tree.utils.goal_conversions import behavior_to_pose_goal
from . import Move
from . import MoveJoint

from riro_srvs.srv import StringGoalStatus
import py_trees.console as console

## import std_msgs.msg as std_msgs
## from actionlib_msgs.msg import GoalStatus
## from control_msgs.msg import FollowJointTrajectoryResult

## from complex_action_client.srv import String_Int, None_String


class MOVEP(Move.MOVE):
    """
    Move to a Cartesian pose
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None, timeout=1, robot_name=None):
        super(MOVEP, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal,
                                   timeout=timeout,
                                   robot_name=robot_name)

        self.blackboard.register_key(key=self.action_goal['pose'], \
                                     access=py_trees.common.Access.READ)

    def make_command(self, uuid=None, enable_wait=False):
        """
        Export this pose move as a complex action client command dictionary.
        """
        # Encode the resolved pose with the same schema used by update().
        return self._make_command(
            "movePose", json.dumps(behavior_to_pose_goal(self)), uuid=uuid, 
            enable_wait=enable_wait,
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
            self.feedback_message = "Sending a pose goal"
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


        
class MOVES(Move.MOVE):
    """
    Move to a Cartesian pose following a straight pose trajectory
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None, timeout=1.,
                 check_contact=False, robot_name=None, gripper=None,
                 gripper_ramp_param=None, claim=None, filter_size=None):
        """
        Args:
            gripper, gripper_ramp_param, claim: exactly as ``MoveJoint.MOVEJ``
                takes them, and for the same reasons -- a straight move leaves
                through the same slot, under the same mixer, and a chain file's
                step states its seam the same way whichever kind it is. None
                for `gripper` OMITS the column (rule i); `claim` is merged last
                by ``Move.MOVE._make_command``, so a file's own ramp wins over
                the node parameter's.
            filter_size: the width of the moving average trajectory_manager
                runs over this move's IK samples. None leaves the field out,
                and the generator then uses its own default of 15 -- which
                rounds both ENDS of the line. A chain file's straight step
                sends 1: straightness is the point.
        """
        super(MOVES, self).__init__(name=name,
                                   action_client=action_client,
                                    action_goal=action_goal,
                                    timeout=timeout,
                                    robot_name=robot_name,
                                    claim=claim)
        self.check_contact = check_contact
        self.gripper = gripper
        self.gripper_ramp_param = gripper_ramp_param
        self.filter_size = filter_size
        # Only a blackboard KEY is registered. `behavior_to_pose_goal` already
        # accepts an inline `geometry_msgs.msg.Pose` -- a chain file's straight
        # step is a pose written down, with no producer to read it from -- and
        # handing that object to `register_key` asks the blackboard for a
        # variable named by a Pose.
        if not isinstance(self.action_goal['pose'], geometry_msgs.msg.Pose):
            self.blackboard.register_key(key=self.action_goal['pose'], \
                                         access=py_trees.common.Access.READ)

    # The same live read `MOVEJ` makes, and it must be the same one: the
    # composite that chains this motion reads that parameter live, and two
    # numbers for one seam is a gripper arriving somewhere nobody stated.
    gripper_ramp_fraction = MoveJoint.MOVEJ.gripper_ramp_fraction

    def make_command(self, uuid=None, enable_wait=False):
        """
        Export this straight pose move as a complex action client command dictionary.
        """
        # Beside the goal, not inside it: the goal is the line the planner
        # plans, and the jaw column is one the planner does not own. Identical
        # to `MOVEJ.make_command` because it is the same column on the same
        # trajectory -- cac appends it in `_append_passthrough_columns`.
        carried = {}
        if self.gripper is not None:
            carried["gripper"] = float(self.gripper)
            fraction = self.gripper_ramp_fraction()
            if fraction is not None:
                carried["passthrough_ramp_fraction"] = fraction
        if self.filter_size is not None:
            carried["filter_size"] = int(self.filter_size)
        # Include contact checking because complex action client forwards it.
        return self._make_command(
            "movePoseStraight", json.dumps(behavior_to_pose_goal(self)), uuid=uuid,
            enable_wait=enable_wait, check_contact=self.check_contact,
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


class MOVEPR(Move.MOVE):
    """
    Move Pose Relative with a certain frame
    
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """
    def __init__(self, name, action_client, action_goal=None, cont=False, timeout=3., robot_name=None):
        super(MOVEPR, self).__init__(name=name,
                                   action_client=action_client,
                                     action_goal=action_goal,
                                     timeout=timeout,
                                     robot_name=robot_name)

        ## self.blackboard.register_key(key=self.action_goal['pose'], \
        ##                              access=py_trees.common.Access.READ)
                                     
        # Enable continuous motion
        self.action_cont = cont

    def make_command(self, uuid=None, enable_wait=False):
        """
        Export this relative pose move as a complex action client command dictionary.
        """
        # Include the relative frame required by trajectory_manager.
        return self._make_command(
            "movePoseRelative", json.dumps(behavior_to_pose_goal(self)), uuid=uuid, 
            enable_wait=enable_wait, frame=self.action_goal['frame'],
        )


    def setup(self, timeout):

        super.setup(timeout)

        if self.action_cont:
            timeout_scale = 0.5
        else:
            timeout_scale = 1.
        self.cmd_req(json.dumps({'action_type': 'setSpeed', 'goal': timeout_scale}))        
        return True


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal or (self.action_cont and self.action_goal['pose'] is not None):
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


class MOVEPROOT(Move.MOVE):
    """
    Move the root joint of the manipulator toward pick-and-place of a target object (pose).

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None, timeout=3., robot_name=None):
        super(MOVEPROOT, self).__init__(name=name,
                                        action_client=action_client,
                                        action_goal=action_goal,
                                        timeout=timeout,
                                        robot_name=robot_name)
    
        self.blackboard.register_key(key=self.action_goal['pose'], \
                                     access=py_trees.common.Access.READ)

    def make_command(self, uuid=None, enable_wait=False):
        """
        Export this root pose move as a complex action client command dictionary.
        """
        # Encode the root pose command for blend generation.
        return self._make_command(
            "movePoseRoot", json.dumps(behavior_to_pose_goal(self)), uuid=uuid,
            enable_wait=enable_wait,
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
            
