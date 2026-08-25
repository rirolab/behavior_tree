import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
from behavior_tree.utils.goal_conversions import behavior_to_pose_goal
from . import Move

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
            return py_trees.Status.FAILURE

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

    def __init__(self, name, action_client, action_goal=None, timeout=1., check_contact=False, robot_name=None):
        super(MOVES, self).__init__(name=name,
                                   action_client=action_client,
                                    action_goal=action_goal,
                                    timeout=timeout,
                                    robot_name=robot_name)
        self.check_contact = check_contact
        self.blackboard.register_key(key=self.action_goal['pose'], \
                                     access=py_trees.common.Access.READ)

    def make_command(self, uuid=None, enable_wait=False):
        """
        Export this straight pose move as a complex action client command dictionary.
        """
        # Include contact checking because complex action client forwards it.
        return self._make_command(
            "movePoseStraight", json.dumps(behavior_to_pose_goal(self)), uuid=uuid, 
            enable_wait=enable_wait, check_contact=self.check_contact,
        )

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

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
            return py_trees.Status.FAILURE

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
            return py_trees.Status.FAILURE

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
            
