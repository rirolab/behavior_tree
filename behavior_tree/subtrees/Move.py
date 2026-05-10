import typing
import json

import rclpy
from action_msgs.msg import GoalStatus
from riro_srvs.srv import StringGoalStatus

import py_trees
from py_trees_ros import exceptions, utilities
import py_trees.console as console
## from rclpy.callback_groups import ReentrantCallbackGroup

class MOVE(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(
        self,
        name,
        action_client,
        action_goal=None,
        timeout=1,
        robot_name=None,
        goal_channel="arm",
    ):
        """
        Initialise a robot action command behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            action_client (:class:`~rclpy.client.Client`): robot command client.
            action_goal: command payload for the robot action.
            timeout (:obj:`float`): command timeout in seconds.
            robot_name (:obj:`str`): optional robot namespace.
            goal_channel (:obj:`str`): action status channel, e.g. ``arm`` or
                ``gripper``.
        """
        super(MOVE, self).__init__(name=name)

        self.arm           = None
        self.robot_name    = robot_name
        self.goal_channel  = goal_channel
        self.action_goal   = action_goal
        self.sent_goal     = False
        self.cmd_req       = action_client
        self.goal_uuid_des = None
        ## self.goal_id       = None
        ## self.goal_status   = None
        self.timeout       = timeout

        # Namespacing lets multi-robot trees keep goal state isolated per arm.
        self.blackboard = self.attach_blackboard_client(
            name=self.name,
            namespace=self.robot_name,
        )
        ## self.blackboard = py_trees.blackboard.Client()
        ## self.callback_group = ReentrantCallbackGroup() 
        self.goal_id_key = f"{self.goal_channel}/goal_id"
        self.goal_status_key = f"{self.goal_channel}/goal_status"
        self.blackboard.register_key(
            key=self.goal_id_key,
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key=self.goal_status_key,
            access=py_trees.common.Access.READ,
        )
        
    def setup(self, node):
        """ """
        self.feedback_message = f"{self.name}: setup"
        ## self.node = node
        ## self.node.create_subscription(GoalStatus, 'arm_client/goal_status',
        ##                              self.goal_status_callback,
        ##                              10,
        ##                              callback_group=self.callback_group)


    def initialise(self):
        """ """
        self.logger.debug(f"{self.__class__.__name__}.initialise()")
        self.sent_goal = False


    def update(self):
        """ """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.sent_goal = True
        return py_trees.common.Status.SUCCESS

    def current_goal_id(self):
        """
        Read the current goal id from the blackboard.

        Returns:
            goal id value or :obj:`None` if it is not available.
        """
        try:
            return self.blackboard.get(self.goal_id_key)
        except KeyError:
            return None

    def current_goal_status(self):
        """
        Read the current goal status from the blackboard.

        Returns:
            goal status value or :obj:`None` if it is not available.
        """
        try:
            return self.blackboard.get(self.goal_status_key)
        except KeyError:
            return None

    def goal_matches_blackboard(self):
        """
        Check whether the sent goal id matches the blackboard goal id.

        Returns:
            :obj:`bool`: whether the current status belongs to this behaviour.
        """
        goal_id = self.current_goal_id()
        if goal_id is None or self.goal_uuid_des is None:
            return False
        match = self.goal_uuid_des == goal_id
        return match.all() if hasattr(match, "all") else bool(match)

    
    def terminate(self, new_status):
        """ """
        self.logger.debug("%s.terminate()" % self.__class__.__name__)
        if self.current_goal_id() is None:
            self.feedback_message = "goal_id is not available"
            return py_trees.common.Status.SUCCESS
        
        status = self.current_goal_status()
        self.logger.debug(f"self.goal_status")
        if self.goal_matches_blackboard() and \
          status in [
              GoalStatus.STATUS_ACCEPTED,
              GoalStatus.STATUS_EXECUTING,
          ]:
            req = StringGoalStatus.Request()
            req.data = json.dumps({'action_type': 'cancel_goal',
                                   'enable_wait': True})
            self.future = self.cmd_req.call_async( req )
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))            
        return py_trees.common.Status.SUCCESS

    ## def goal_status_callback(self, msg):
    ##     console.loginfo(f"{str(msg)}")
    ##     self.goal_id     = msg.goal_info.goal_id
    ##     self.goal_status = msg.status
        
