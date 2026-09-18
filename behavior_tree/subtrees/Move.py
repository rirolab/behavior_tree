import typing
import json
import time

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
                 robot_name=None, goal_channel="arm", claim=None):
        """
        Initialise a robot action command behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            action_client (:class:`~rclpy.client.Client`): robot command client.
            action_goal: command payload for the robot action.
            timeout (:obj:`float`): command timeout in seconds.
            robot_name (:obj:`str`): optional robot namespace.
            goal_channel (:obj:`str`): action status channel, ``arm`` or
                ``gripper``, or ``None`` when the tree does not name one. The
                real-robot line had dropped this since its arm_client published
                one status topic for everything; the merged arm_client splits it
                per channel, so it is load-bearing again. ``None`` means the
                route is the client's to decide and the behaviour watches both
                channels -- see ``status_channels``.
            claim: this motion's own request fields, or None -- the seam it
                asks the mixer for (``claim_progress_threshold``,
                ``claim_closure``, ``claim_smoothing``,
                ``claim_passthrough_only``, popped by
                ``_stream_client.CLAIM_KWARGS``), how it is planned
                (``plan_from_arm``), and the span a carried gripper column
                moves over (``passthrough_ramp`` /
                ``passthrough_ramp_fraction``). Merged into the command LAST,
                so a motion that states a field beats the node-level default
                the behaviour would otherwise compute for it.

                A dict rather than named arguments, and read by no behaviour:
                cac forwards every key it does not own
                (``_action_server.py`` ``rejected_command_keys``), so the tree
                does not have to learn a field to pass one on. It is per
                MOTION, not a ``MoveBlend`` attribute -- the composite chains
                children and knows nothing about what any of them claims; the
                mixer takes the seam from the incoming motion's own claim.
        """
        super(MOVE, self).__init__(name=name)

        self.arm           = None
        self.robot_name    = robot_name
        self.goal_channel  = goal_channel
        self.action_goal   = action_goal
        self.claim         = dict(claim) if claim else None
        # When this motion's request left, and when the motion stopped. Two
        # facts a chain has to have to say what a run did -- the window the jaw
        # verdict is taken over runs from the end of the last policy to the
        # dispatch of the step that releases -- and a motion is the only thing
        # that knows either. Stamped here rather than at each of the dozen
        # `sent_goal = True` lines, so a behaviour added later is stamped too.
        self._sent_goal    = False
        self.dispatched_at = None
        self.finished_at   = None
        self.sent_goal     = False
        self.cmd_req       = action_client
        self.future        = None
        self.goal_uuid_des = None
        ## self.goal_id       = None
        ## self.goal_status   = None
        self.timeout       = timeout
        # The ROS node, once `setup` runs. None until then, and None in a unit
        # test, so everything that reads it has to work without one.
        self._node         = None

        # Which status channels can report this goal.
        #
        # A behaviour that names a channel gets that one. A behaviour that names
        # none watches both and claims whichever one carries its own goal id --
        # which is the only way to stay out of the wiring: whether a gripper
        # command leaves on the arm controller's stream or on a gripper server
        # is the client's answer (`ArmClient._gripper_writer`), and the status
        # appears on the channel the motion actually took. Deciding it here
        # would mean the tree had learned the deployment.
        self.status_channels = ((self.goal_channel,) if self.goal_channel
                                else ("arm", "gripper"))

        # Under overlap a motion is streamed and its status arrives on one of
        # the two slot channels (arm/a, arm/b) rather than the single arm
        # channel, because two arm motions are live at once and one channel
        # cannot represent both. The behaviour does not know in advance which
        # slot it will land in, so it watches both and claims whichever one is
        # carrying its own goal id. Slots exist on the arm channel only -- a
        # gripper server is never streamed -- so a behaviour watching both
        # channels still has exactly one slot pair to search.
        self.slot_channel = "arm" if "arm" in self.status_channels else None
        self.stream_slots = ("a", "b") if self.slot_channel else ()

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
        self.channel_keys = {}
        for channel in self.status_channels:
            keys = {
                "goal_id": f"{channel}/goal_id",
                "goal_status": f"{channel}/goal_status",
            }
            self.channel_keys[channel] = keys
            for key in keys.values():
                self.blackboard.register_key(
                    key=key, access=py_trees.common.Access.READ)
        # The first channel's pair, kept under the old names: it is what a
        # single-channel behaviour reads, and what a two-channel one falls back
        # to before either channel has reported anything of its own.
        first = self.status_channels[0]
        self.goal_id_key = self.channel_keys[first]["goal_id"]
        self.goal_status_key = self.channel_keys[first]["goal_status"]

        # Slot channels: goal_id, goal_status, and progress per slot.
        self.slot_keys = {}
        for slot in self.stream_slots:
            keys = {
                "goal_id": f"{self.slot_channel}/{slot}/goal_id",
                "goal_status": f"{self.slot_channel}/{slot}/goal_status",
                "progress": f"{self.slot_channel}/{slot}/progress",
                # Whether the producer in this slot says its motion may be
                # handed over from. Only a producer that HAS a predicate
                # publishes it, so this key is usually absent and `_read`
                # answers None -- "no answer", which is not "no".
                "handover": f"{self.slot_channel}/{slot}/handover",
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

    @property
    def sent_goal(self):
        """Whether this behaviour's request has gone out."""
        return self._sent_goal

    @sent_goal.setter
    def sent_goal(self, value):
        value = bool(value)
        if value and not self._sent_goal:
            self.dispatched_at = self.now()
        self._sent_goal = value

    def now(self):
        """The clock every stamp and every trace in this run shares.

        The node's, once there is one. Without a node -- a unit test -- the
        wall clock, which is the same clock a node without `use_sim_time` reads
        anyway.
        """
        if self._node is None:
            return time.time()
        return self._node.get_clock().now().nanoseconds * 1e-9

    def requested_timeout(self):
        """The `timeout` this request asks for, resolved at DISPATCH.

        A number for every motion whose duration is known when the subtree is
        built, and a callable for one whose is not: a step planned from the
        measured arm -- the step after a policy, which ends where it ends --
        cannot be paced until it is sent, so its duration arrives here as the
        question rather than the answer. Asked once per request, where the
        answer is still true.

        The callable is the caller's and must not raise; a pacing number is not
        worth a tree.
        """
        timeout = self.timeout
        return timeout() if callable(timeout) else timeout

    def _make_command(self, action_type, goal, uuid=None, enable_wait=False, **kwargs):
        """
        Build the complex action client command dictionary for blend composition.
        """
        # Preserve the routing fields needed by complex action client and trajectory_manager.
        command = {
            "action_type": action_type,
            "goal": goal,
            "timeout": self.requested_timeout(),
            "enable_wait": enable_wait,
        }
        # Omitted when the tree names no channel, which is not the same as
        # naming a wrong one: cac reads an absent `goal_channel` as "you decide"
        # and answers from the writer it actually has.
        if self.goal_channel is not None:
            command["goal_channel"] = self.goal_channel
        if uuid is not None:
            command["uuid"] = uuid
        if self.robot_name is not None:
            command["robot_name"] = self.robot_name
        command.update(kwargs)
        # Last, so this motion's own statement wins over a default a subclass
        # computed for it -- a step whose chain file names its ramp is not
        # overridden by the tree's `gripper_ramp` node parameter.
        if self.claim:
            command.update(self.claim)
        return command
        
    def setup(self, node):
        """ """
        # Kept, as `MoveBlend.setup` keeps it: a behaviour that has to agree
        # with the composite chaining it -- on the threshold the seam opens at,
        # say -- reads the same live node parameter the composite reads, rather
        # than being handed a copy that freezes at startup.
        self._node = node
        self.feedback_message = f"{self.name}: setup"
        ## self.node = node
        ## self.node.create_subscription(GoalStatus, 'arm_client/goal_status',
        ##                              self.goal_status_callback,
        ##                              10,
        ##                              callback_group=self.callback_group)


    def initialise(self):
        """ """
        self.logger.debug(f"{self.__class__.__name__}.initialise()")
        self.dispatched_at = None
        self.finished_at = None
        self._sent_goal = False
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

    def _owning_channel(self):
        """The plain status channel carrying this behaviour's goal id, or None.

        The same uuid match as `_owning_slot`, one level out: a behaviour that
        named no channel is told where its motion went by finding itself there.
        A behaviour that named one searches a tuple of length one, so this is
        the identity for every other subclass.
        """
        for channel in self.status_channels:
            if self._uuid_matches(self._read(self.channel_keys[channel]["goal_id"])):
                return channel
        return None

    def current_goal_id(self):
        """Goal id for this behaviour: from its slot if streamed, else from
        whichever watched channel reports it, else whatever the first one holds."""
        slot = self._owning_slot()
        if slot is not None:
            return self._read(self.slot_keys[slot]["goal_id"])
        channel = self._owning_channel()
        if channel is not None:
            return self._read(self.channel_keys[channel]["goal_id"])
        return self._read(self.goal_id_key)

    def current_goal_status(self):
        """Goal status for this behaviour, from whichever channel carries it."""
        slot = self._owning_slot()
        if slot is not None:
            return self._read(self.slot_keys[slot]["goal_status"])
        channel = self._owning_channel()
        if channel is not None:
            return self._read(self.channel_keys[channel]["goal_status"])
        return self._read(self.goal_status_key)

    def cancel_channel(self):
        """The channel a cancel for this goal must name.

        The one its status came back on, since that is the channel cac is
        driving it from. A slot is the arm channel's, so a streamed motion
        cancels the arm. Falls back to the declared channel when nothing carries
        the goal -- `terminate` only sends a cancel once something does, so the
        fallback is for callers rather than for the cancel itself.
        """
        if self._owning_slot() is not None:
            return self.slot_channel
        channel = self._owning_channel()
        return channel if channel is not None else self.goal_channel

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

    def handover_ready(self):
        """The producer's own "ready to be handed over from", or None.

        None when nothing publishes it, which is every planned trajectory: a
        primitive's readiness IS its progress, and there is no second question
        to ask about it.
        """
        slot = self._owning_slot()
        if slot is None:
            return None
        return self._read(self.slot_keys[slot]["handover"])

    def ready_for_successor(self, threshold, dispatch_on_start=False):
        """May the NEXT motion be dispatched now?

        The composite chaining motions asks this of the one that is running,
        instead of testing its progress itself. Two behaviours answer it
        differently and the difference is not a threshold:

          * a planned trajectory ends where it said it would, so its successor
            can be planned from its target at any time -- which is what
            `dispatch_on_start` buys, paying the plan's 0.2-0.44 s while this
            motion still runs.
          * a policy ends where it ends. A successor dispatched at progress > 0
            of a policy is planned from an arm nobody knows, so MOVEBYPOLICY
            overrides this and refuses until its container says the arm has
            stopped (`Policy.MOVEBYPOLICY.ready_for_successor`).

        The default is the test this replaced, unchanged: progress at or past
        the threshold, with a SUCCESS counting as 1.0 whatever the topics say.
        """
        if self.status == py_trees.common.Status.SUCCESS:
            return True
        progress = self.current_progress()
        if progress is None:
            return False
        if dispatch_on_start:
            return progress > 0.0
        return progress >= threshold

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
        # When this motion stopped, whichever way it stopped. The chain's jaw
        # window opens at the end of the last policy, and nothing else in the
        # tree records that instant.
        if self.finished_at is None and new_status in (
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.FAILURE):
            self.finished_at = self.now()
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
            # down with it. The channel is the one this goal's status arrived
            # on, not a declared one, so a command whose route cac chose is
            # cancelled where it actually went.
            req.data = json.dumps({'action_type': 'cancel_goal',
                                   'goal_channel': self.cancel_channel(),
                                   'enable_wait': True})
            self.future = self.cmd_req.call_async( req )
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))            
        return py_trees.common.Status.SUCCESS

    ## def goal_status_callback(self, msg):
    ##     console.loginfo(f"{str(msg)}")
    ##     self.goal_id     = msg.goal_info.goal_id
    ##     self.goal_status = msg.status
        
