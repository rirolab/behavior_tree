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

    def __init__(self, name, action_client, action_goal=None, timeout=1, 
                 robot_name=None, goal_channel="arm"):
        """
        Initialise a robot action command behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            action_client (:class:`~rclpy.client.Client`): robot command client.
            action_goal: command payload for the robot action.
            timeout (:obj:`float`): command timeout in seconds.
            robot_name (:obj:`str`): optional robot namespace.
            goal_channel (:obj:`str`): action status channel, ``arm`` or
                ``gripper``. The real-robot line had dropped this because its
                arm_client published one status topic for everything; the merged
                arm_client splits it per channel, so it is load-bearing again.
        """
        super(MOVE, self).__init__(name=name)

        self.arm           = None
        self.robot_name    = robot_name
        self.goal_channel  = goal_channel
        self.action_goal   = action_goal
        self.sent_goal     = False
        self.cmd_req       = action_client
        self.future        = None
        self.goal_uuid_des = None
        ## self.goal_id       = None
        ## self.goal_status   = None
        self.timeout       = timeout

        # Under overlap a motion is streamed and its status arrives on one of
        # the two slot channels (arm/a, arm/b) rather than the single arm
        # channel, because two arm motions are live at once and one channel
        # cannot represent both. The behaviour does not know in advance which
        # slot it will land in, so it watches both and claims whichever one is
        # carrying its own goal id. Only used on the arm channel; the gripper is
        # never streamed.
        self.stream_slots = ("a", "b") if self.goal_channel == "arm" else ()

        # Namespacing lets multi-robot trees keep goal state isolated per arm.
        self.blackboard = self.attach_blackboard_client(
            name=self.name,
            namespace=self.robot_name,
        )
        ## self.blackboard = py_trees.blackboard.Client()
        ## self.callback_group = ReentrantCallbackGroup()
        # Per goal channel, not flat. The real line used flat "goal_id" /
        # "goal_status" while arm_client published a single status topic; the
        # merged arm_client publishes one per channel
        # ("arm_client/{arm,gripper}/goal_status") because the arm and the
        # gripper are cancelled and retired independently under overlap. Flat
        # keys here would have the gripper's status overwrite the arm's.
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

        # Slot channels: goal_id, goal_status, and progress per slot.
        self.slot_keys = {}
        for slot in self.stream_slots:
            keys = {
                "goal_id": f"{self.goal_channel}/{slot}/goal_id",
                "goal_status": f"{self.goal_channel}/{slot}/goal_status",
                "progress": f"{self.goal_channel}/{slot}/progress",
            }
            self.slot_keys[slot] = keys
            for key in keys.values():
                self.blackboard.register_key(
                    key=key, access=py_trees.common.Access.READ)

        # May a MoveBlend start the NEXT child before this one finishes?
        #
        # Derived from the slots by default -- a streamed arm motion is blended
        # by the mixer, a gripper op is not -- but kept as its own attribute so
        # a behaviour that is overlappable for a different reason can say so.
        # MOVEBYPOLICY is exactly that: its status arrives on the plain arm
        # channel (one policy goal at a time, so there is nothing to
        # disambiguate) while its command still goes through a mixer slot.
        self.overlappable = bool(self.stream_slots)

    def _make_command(self, action_type, goal, uuid=None, enable_wait=False, **kwargs):
        """
        Build the complex action client command dictionary for blend composition.
        """
        # Preserve the routing fields needed by complex action client and trajectory_manager.
        command = {
            "action_type": action_type,
            "goal": goal,
            "timeout": self.timeout,
            "enable_wait": enable_wait,
            "goal_channel": self.goal_channel,
        }
        if uuid is not None:
            command["uuid"] = uuid
        if self.robot_name is not None:
            command["robot_name"] = self.robot_name
        command.update(kwargs)
        return command
        
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

    def _read(self, key):
        try:
            return self.blackboard.get(key)
        except KeyError:
            return None

    def _uuid_matches(self, goal_id):
        if goal_id is None or self.goal_uuid_des is None:
            return False
        match = self.goal_uuid_des == goal_id
        return match.all() if hasattr(match, "all") else bool(match)

    def _owning_slot(self):
        """The slot channel currently carrying this behaviour's goal id.

        A streamed motion lands in slot a or b; the behaviour finds itself by
        matching its own goal id against each slot's, rather than being told the
        slot. Returns None when no slot carries this goal (yet, or not streamed).
        """
        for slot in self.stream_slots:
            if self._uuid_matches(self._read(self.slot_keys[slot]["goal_id"])):
                return slot
        return None

    def current_goal_id(self):
        """Goal id for this behaviour: from its slot if streamed, else the arm
        channel."""
        slot = self._owning_slot()
        if slot is not None:
            return self._read(self.slot_keys[slot]["goal_id"])
        return self._read(self.goal_id_key)

    def current_goal_status(self):
        """Goal status for this behaviour, from whichever channel carries it."""
        slot = self._owning_slot()
        if slot is not None:
            return self._read(self.slot_keys[slot]["goal_status"])
        return self._read(self.goal_status_key)

    def current_progress(self):
        """Fraction of this motion completed, in [0, 1], or None.

        Only streamed motions report progress; a motion sent to the controller
        as a goal has none, so this returns None off the streaming path -- an
        MoveBlend then simply cannot overlap it, and falls back to
        sequential.
        """
        slot = self._owning_slot()
        if slot is None:
            return None
        return self._read(self.slot_keys[slot]["progress"])

    def goal_matches_blackboard(self):
        """
        Check whether the sent goal id matches the blackboard goal id.

        Returns:
            :obj:`bool`: whether the current status belongs to this behaviour.
        """
        return self._uuid_matches(self.current_goal_id())

    def command_response_status(self):
        """
        Convert immediate complex action client failures into a behavior result.

        Returns:
            :class:`py_trees.common.Status` or :obj:`None` when status topics
            should continue deciding the command result.
        """
        # Wait until complex action client has replied.
        if self.future is None or not self.future.done():
            return None

        # Treat complex action client exceptions as behavior failures.
        try:
            response = self.future.result()
        except Exception as exc:
            self.feedback_message = f"command service failed: {exc}"
            return py_trees.common.Status.FAILURE

        # Ignore empty or non-terminal responses; goal status topics finish normal goals.
        goal_status = getattr(response, "goal_status", None)
        if goal_status is None:
            return None
        status = getattr(goal_status, "status", GoalStatus.STATUS_UNKNOWN)
        failure_statuses = [
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_UNKNOWN,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_CANCELED,
        ]
        if status not in failure_statuses:
            return None

        # Match service response UUID to this behavior's requested goal.
        response_uuid = list(getattr(goal_status.goal_info.goal_id, "uuid", []))
        if self.goal_uuid_des is not None and response_uuid:
            if response_uuid != list(self.goal_uuid_des):
                return None
        elif self.goal_uuid_des is not None:
            return None

        # Fail immediately when complex action client rejected the controller goal.
        self.feedback_message = "FAILURE"
        return py_trees.common.Status.FAILURE

    
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
            # The channel matters: cac cancels every channel when it is not
            # told one, and under overlap retiring the outgoing arm motion
            # would take the gripper -- and the incoming motion's hold on it --
            # down with it.
            req.data = json.dumps({'action_type': 'cancel_goal',
                                   'goal_channel': self.goal_channel,
                                   'enable_wait': True})
            self.future = self.cmd_req.call_async( req )
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))            
        return py_trees.common.Status.SUCCESS

    ## def goal_status_callback(self, msg):
    ##     console.loginfo(f"{str(msg)}")
    ##     self.goal_id     = msg.goal_info.goal_id
    ##     self.goal_status = msg.status
        
