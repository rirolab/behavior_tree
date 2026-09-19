import json

import numpy as np
import py_trees
from action_msgs.msg import GoalStatus

from riro_srvs.srv import StringGoalStatus

from . import Move
from .MoveBlend import (
    OVERLAP_DISPATCH_ON_START_PARAM,
    OVERLAP_THRESHOLD_PARAM,
    MoveBlend,
)


class MOVEBYPOLICY(Move.MOVE):
    """
    Execute policy through complex_action_client.
    """

    def __init__(self, name, action_client, action_goal=None, timeout=5.0,
                 robot_name=None, end_handover=None, early_retreat=False):
        """
        Initialise a policy execution behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            action_client (:class:`~rclpy.client.Client`): robot command client.
            action_goal (:obj:`dict`): policy command payload.
            timeout (:obj:`float`): command timeout in seconds.
            robot_name (:obj:`str`): optional robot namespace.
            end_handover (:obj:`dict`): this skill's ``end_handover`` as its
                container describes it -- ``progress_threshold`` (where the next
                motion may OPEN its seam) and, when the skill declares one,
                ``still_from`` (where it stops MOVING THE ARM, which is where
                the next motion may be DISPATCHED). Read at build time from
                ``skill_facts(skill_id)``, the same dict the chain driver reads.
                None when the container describes none, and the successor then
                waits for the terminal.
            early_retreat (:obj:`bool`): whether this step's file allows the
                next motion to be dispatched before this policy's terminal at
                all. False keeps the sequential behaviour whatever the
                container declares -- it is the chain file's per-step switch,
                not a property of the skill.
        """
        super(MOVEBYPOLICY, self).__init__(
            name=name,
            action_client=action_client,
            action_goal=action_goal,
            timeout=timeout,
            robot_name=robot_name,
        )

        self.end_handover = dict(end_handover) if end_handover else None
        self.early_retreat = bool(early_retreat)

        # Has THIS episode been seen below its own floor yet? See
        # `ready_for_successor`. False until a progress of this motion's has
        # been read, which is what separates a running episode from the
        # terminal value the previous one left on the slot's keys.
        self._seen_below_floor = False

        # A policy step is a slot motion, and is found the way every other one
        # is: by matching its own goal id against each slot's.
        #
        # It used to declare `stream_slots = ()` and read a hard-wired
        # `policy/progress` key fed from slot b, which was true only of a
        # deployment with one policy container whose motion always landed
        # there. A slot-mode policy is reported by the CONTAINER on whichever
        # slot it claimed (`policy_slot_node.py` publishes goal_status and
        # progress on its slot; `mixer_slot.py` serves them), and cac
        # deliberately publishes nothing for it
        # (`_policy_manager_client.py`) -- so a tree reading slot b saw
        # nothing at all whenever the container was granted slot a, and could
        # not overlap the step. Inheriting `Move.MOVE`'s slot matching is both
        # the fix and one fewer thing for the tree to know.
        #
        # `overlappable` follows the slots, as it does everywhere else, so it
        # needs no statement here either.

        self.logger.debug("%s.__init__()" % self.__class__.__name__)

    #: The only statuses a PLAIN CHANNEL report may finish a policy step with.
    #: Read by `current_goal_status`; see its docstring for why SUCCEEDED is not
    #: among them.
    CHANNEL_FAILURES = (
        GoalStatus.STATUS_ABORTED,
        GoalStatus.STATUS_CANCELING,
        GoalStatus.STATUS_CANCELED,
    )

    def current_goal_status(self):
        """This policy's status -- from its SLOT, or a refusal from the channel.

        The inherited reader takes whichever source carries this behaviour's
        goal id, slot first and the plain channel after it, and for a policy the
        second half of that is not safe. cac says so itself: a slot-mode policy
        does not own the arm channel (`_policy_manager_client`
        `_start_policy_controller_mode` writes it only for a policy this node
        publishes itself, and `_finish_policy_request` commits a terminal only
        for one that `owns_channel`). The container reports this motion on the
        slot the mixer granted it (`policy_slot_node.py`, through
        `mixer_slot.py`), and that is the one place its SUCCESS can come from.

        So a channel-level status may FAIL this step and may not finish it:

          * ABORTED / CANCELED / CANCELING is the refusal path, and it has to
            keep working. cac aborts a policy before any slot exists -- a
            request policy_manager will not take, a worker that raises -- and
            writes (this goal's id, ABORTED) on the channel together
            (`_action_utils._aborted_goal_status`). Nothing else will ever tell
            the tree, so dropping it would hang the step until its timeout.
          * anything else, SUCCEEDED included, is not this motion's answer and
            reads as "no answer yet", which leaves the step RUNNING until its
            slot reports.

        Defence in depth, and the depth is the point: cac keeps the channel's
        goal id and status paired so a foreign terminal cannot carry this
        policy's id at all, and this makes the tree refuse the one that would
        matter even if it did. TREE2B measured what the missing half costs -- a
        policy step finishing 0.12 s after its goal went out, before the mixer
        had granted the container a slot, every step behind it dispatched at
        once and three motions dropped for want of a free slot.
        """
        slot = self._owning_slot()
        if slot is not None:
            return self._read(self.slot_keys[slot]["goal_status"])
        channel = self._owning_channel()
        if channel is not None:
            status = self._read(self.channel_keys[channel]["goal_status"])
            return status if status in self.CHANNEL_FAILURES else None
        # No fallback to the first channel's key. The inherited one reads a
        # status nothing has matched an id against, which for a policy is
        # another motion's by construction.
        return None

    def dispatch_floor(self):
        """The progress at which the next motion may be DISPATCHED, or None.

        ``still_from`` when the skill declares one -- where it stops moving the
        arm -- and its ``progress_threshold`` otherwise. Two different floors
        for two different questions, and the declared one is deliberately the
        lower of the two on every skill that states it (cube_diag: 0.61 against
        0.70), because dispatching is what costs the plan's 0.2-0.44 s and that
        has to be paid BEFORE the seam may open, not after.
        """
        # `getattr`: `update` asks every tick now, and a behaviour assembled
        # without `__init__` -- a test double for the refusal path -- has no
        # `end_handover` at all. "No declaration" and "not constructed" are the
        # same answer here, which is None.
        declared = getattr(self, "end_handover", None)
        if not declared:
            return None
        floor = declared.get("still_from")
        if floor is None:
            floor = declared.get("progress_threshold")
        return None if floor is None else float(floor)

    def ready_for_successor(self, threshold, dispatch_on_start=False):
        """May the motion after this policy be dispatched now?

        Not the default test, and `dispatch_on_start` is deliberately ignored:
        a primitive ends at the target it was planned to, so its successor can
        be planned from that target at any time, while a policy ends where it
        ends. A retreat dispatched at progress > 0 of a policy is planned from
        an arm nobody knows -- it is planned from the MEASURED arm, mid-episode,
        which is not where the policy will leave it.

        Two terms, and the second is the container's, not the tree's:

          * progress at or past `dispatch_floor()`, the fraction the skill
            declared. Nobody can observe that fraction in time -- certifying
            stillness costs `still_for` seconds after the arm stops, and a
            policy stops 0.3-0.5 s before its own terminal.
          * the container's handover flag, which is its own predicate over its
            own commanded window and the measured jaws
            (`policy_slot_node._handover_ready`). One predicate, declared beside
            the skill; a tree evaluating a second one of its own tolerance
            would be a second opinion rather than a veto.

        A SUCCESS admits regardless -- that is the terminal, and it is the ONLY
        thing that admits when the file did not ask for an early retreat, or
        when the container describes no handover at all. Note what is not here:
        `progress >= 1.0`. A policy's progress reaches 1.0 at its last frame
        while its weight is still leaving (the mixer retires it at the fade's
        end, `policy_slot: finished`), so 1.0 is not the terminal.
        """
        if self.status == py_trees.common.Status.SUCCESS:
            return True
        if not self.early_retreat:
            return False
        floor = self.dispatch_floor()
        if floor is None:
            return False
        progress = self._observe_progress(floor)
        if progress is None:
            return False
        if progress < floor:
            return False
        if not getattr(self, "_seen_below_floor", False):
            # Past the floor without ever having been below it: the slot's
            # progress and handover keys do not belong to this motion.
            #
            # A slot is matched by goal id (`Move._owning_slot`), but progress
            # and handover arrive on two SEPARATE topics that carry no goal id
            # (`multi_dynamic_behavior_tree` feeds three independent
            # `ToBlackboard` subscribers per slot), and the container publishes
            # them only while a motion runs. So between one episode's end and
            # the next one's first publish, both keys hold the PREVIOUS
            # episode's terminal values -- progress ~1.0 and handover True --
            # and the tick where the new goal id lands first reads them as this
            # episode's. The retreat was then dispatched at the policy's START,
            # planned from an arm the policy then drove for the rest of its
            # episode; the mixer held it 25.6 s and started it against a chain
            # point 1.45 rad away (TREE2B8 6, TREE2B9 2). Measured at 1 of 16
            # episodes -- a per-tick delivery race, not a fixed window.
            #
            # An episode is below its floor for `still_from` of its length
            # (22.5 s of 28.2 s here), so no tree rate can miss it; a policy
            # short enough to cross the floor inside one tick would never admit
            # early, which is the safe direction.
            return False
        # `is True`, not truthiness: None is "the producer publishes no
        # predicate", which must not read as ready.
        return self.handover_ready() is True

    def _observe_progress(self, floor):
        """This step's progress, latching whether the episode has been seen
        running -- below its own floor -- since `initialise`.

        Called from `update` as well as from `ready_for_successor`, so the
        latch follows the TICK rather than whoever happens to poll the
        predicate: a composite that only asks near the end of a motion would
        otherwise never see the episode below its floor, and nothing would
        admit at all.
        """
        if floor is None:
            # Nothing to latch against, and nothing that reads the latch can
            # admit either (`ready_for_successor` refuses on a floor of None).
            # Checked BEFORE `current_progress`, which reads the slot feeds a
            # behaviour assembled without `__init__` does not have.
            return None
        progress = self.current_progress()
        if progress is not None and progress < floor:
            self._seen_below_floor = True
        return progress

    def initialise(self):
        """A fresh episode has not been seen running yet."""
        super(MOVEBYPOLICY, self).initialise()
        self._seen_below_floor = False

    def update(self):
        """
        Send the policy command and wait for the matching goal status.

        Returns:
            :class:`~py_trees.common.Status`: behaviour status.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Every tick, so the "seen running" latch `ready_for_successor` reads
        # does not depend on who polls that predicate or how often.
        self._observe_progress(self.dispatch_floor())

        if self.cmd_req is None:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16, dtype=np.uint8)
            cmd_str = json.dumps(
                {
                    "action_type": "moveByPolicy",
                    "goal": self.action_goal,
                    "uuid": self.goal_uuid_des.tolist(),
                    "timeout": self.timeout,
                    "enable_wait": False,
                }
            )
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)

            self.sent_goal = True
            self.feedback_message = "Sending a policy goal"
            return py_trees.common.Status.RUNNING

        if self.current_goal_id() is None:
            return py_trees.common.Status.RUNNING

        if not self.goal_matches_blackboard():
            return py_trees.common.Status.RUNNING

        status = self.current_goal_status()
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.SUCCESS, \
                                   self.feedback_message))
            return py_trees.common.Status.SUCCESS

        if status in [
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_CANCELED,
        ]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                   self.feedback_message))
            return py_trees.common.Status.FAILURE

        self.feedback_message = "running"
        return py_trees.common.Status.RUNNING

    def make_command(self, uuid=None, enable_wait=False):
        """
        Export this policy execution as a complex action client command dictionary.
        """
        # Encode policy execution for MoveBlend composition.
        return self._make_command(
            "moveByPolicy", self.action_goal, uuid=uuid, enable_wait=enable_wait,
        )


def create_subtree(action_client, step_goal, **kwargs):
    """
    Create a policy execution subtree for one grounding step.

    An MoveBlend rather than a plain Sequence, and not for its own sake --
    with one child there is nothing here to overlap. It is so this root answers
    `current_progress()` and `overlappable` for the sequence that chains the
    steps: that is what lets a primitive start blending into this policy step,
    and this step into the primitive after it. Inert until the tree's
    `overlap_progress_threshold` drops below 1.0.

    Args:
        action_client (:class:`~rclpy.client.Client`): robot command client.
        step_goal (:obj:`dict`): policy command payload.

    Returns:
       :class:`~py_trees.behaviour.Behaviour`: subtree root
    """
    root = MoveBlend(
        name="Policy",
        threshold_param=OVERLAP_THRESHOLD_PARAM,
        dispatch_param=OVERLAP_DISPATCH_ON_START_PARAM,
        progress_threshold=1.0,
        # The same arm the MOVEBYPOLICY below runs on. The chain that chains
        # these steps asks its children which arm they are for, and
        # `MoveBlend._validate_robots` reads exactly this attribute to refuse a
        # chain mixing two -- so a policy step that answered None could not trip
        # the check and read as unowned to the chain above.
        robot_name=kwargs.get("robot_name"),
    )
    run_policy = MOVEBYPOLICY(
        name="MoveByPolicy",
        action_client=action_client,
        action_goal=step_goal,
        timeout=float(step_goal.get("timeout", 5.0)),
        robot_name=kwargs.get("robot_name"),
        # What the container describes for this skill, and what the chain file
        # asks of this step. Both None/False for a job that knows neither, and
        # the step is then sequential -- its successor waits for the terminal,
        # which is what every policy job did before the hook existed.
        end_handover=kwargs.get("end_handover"),
        early_retreat=kwargs.get("early_retreat", False),
    )
    root.add_child(run_policy)
    return root
