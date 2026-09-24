"""The terminal ledger, merged onto the blackboard.

A slot topic carries ONE (goal id, status) at a time and a reader keeps only
the newest message, so a terminal published there is a fact in the wrong shape:
the motion that produced it has left the slot, the next occupant is already
publishing its own goal id, and whichever of the two the tree read last decides
whether a step ever finishes. Producers used to buy the read with time -- the
arm client re-broadcast a retired motion's terminal for `stream_done_hold` and
stayed silent on a slot a policy had just left for `stream_foreign_done_hold`
-- which delayed every reuse of a slot and still lost the race under sim time,
where both producers run off one `/clock` and the interleaving is fixed for a
whole run rather than random.

So terminals travel on their own channel, `<robot>/stream/terminals`, keyed by
goal id: an `action_msgs/GoalStatusArray` that every producer on that arm
publishes ITS terminals to, re-sent while they are inside the producer's TTL.
One goal id has one terminal, from the producer that ran it (the contract, cac
`_stream_client`).

Four properties of this behaviour follow from there being MANY publishers, and
from each of them re-sending:

  * it MERGES rather than overwrites, and never clears. Each producer sends its
    own subset, so the last array to arrive is not the whole truth -- taking it
    as the whole truth would drop the arm client's terminals every time the
    policy container published, and the other way round.
  * it merges in the CALLBACK, not in `update`. Two publishers' arrays landing
    between two ticks of a 2 Hz tree is the normal case, not the corner, and
    keeping only `self.msg` would discard the first of them.
  * FIRST WRITE WINS. A second terminal for a goal id already held -- another
    status, or the same status from a second producer (a different stamp) --
    is a contract violation. It is kept out and said once per goal id. Taking
    the newest instead made a violation an oscillation: both producers re-send
    every tick, so the value would flip at each callback and the step's answer
    would depend on the phase of the tree's tick.
  * entries age by when they ARRIVED HERE, on this node's clock, and are
    judged against the PREVIOUS tick. The producer's `goal_info.stamp` is
    another node's clock: with `use_sim_time` set differently on the two ends,
    pruning by it dropped an entry in the first `update` before any behaviour
    read it, or never dropped it at all. And against `now`, a tree stalled
    longer than the TTL would discard in its first tick back everything that
    arrived while it was stalled. Against the previous tick, every entry is on
    the blackboard for at least one tick. A producer's own TTL bounds its
    re-sends, so a re-send never refreshes an entry's age here.
"""

import time

import py_trees
import rclpy.qos

from action_msgs.msg import GoalStatusArray

from py_trees_ros import subscribers


#: Seconds an entry stays after it first ARRIVED, counted up to the previous
#: tick. Matches the producers' `terminal_ledger_ttl` default. The producers'
#: TTL is what bounds their re-sends; this one only bounds how long the
#: blackboard keeps a result nobody has asked for.
DEFAULT_TTL = 30.0

#: How many messages the subscription may hold between two ticks.
#:
#: NOT `qos_profile_unlatched()`, which every other feed in this tree uses and
#: which is depth 1. Depth 1 is right for a signal whose newest value is the
#: whole truth -- a progress, a handover flag -- and wrong here for exactly the
#: reason the ledger exists: MANY producers publish to this topic, each its own
#: subset, and a depth-1 queue holds the last message to arrive rather than all
#: of them. Measured on a live graph (domain 79, two publishers, back to back
#: as two producers on one `/clock` are): the depth-1 subscriber saw ONE of the
#: two arrays on every round, always the same one, so the other producer's
#: terminals never reached the blackboard at all -- the deterministic loss the
#: shared slot topic had, moved one hop. The same trial at depth 10 kept both.
#:
#: 20 is two producers' re-sends with room for a tick the executor is late to.
QUEUE_DEPTH = 20

_STATUS_NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                 4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


def uuid_hex(goal_uuid):
    """The key a terminal is filed under: 16 bytes as 32 hex characters.

    Takes whatever a goal id is held as on either side -- a numpy uint8 array
    (`Move.goal_uuid_des`), the array field of a ROS `UUID`, or a plain list --
    since the whole point of the ledger is that a behaviour and a producer that
    never meet agree on the key.
    """
    if goal_uuid is None:
        return None
    raw = getattr(goal_uuid, 'uuid', goal_uuid)
    try:
        return ''.join('{:02x}'.format(int(byte)) for byte in raw)
    except (TypeError, ValueError):
        return None


class ToBlackboard(subscribers.Handler):
    """Merge `<robot>/stream/terminals` into `<robot>/terminals`.

    The blackboard value is a plain dict `{uuid hex: status}`. A behaviour asks
    it for its OWN goal id and nothing else, so the dict never needs to be
    ordered, cleared, or scoped to a slot.
    """

    def __init__(self, name, topic_name, blackboard_variable, ttl=DEFAULT_TTL):
        super(ToBlackboard, self).__init__(
            name=name,
            topic_name=topic_name,
            topic_type=GoalStatusArray,
            # NEVER. The ledger is the one thing on this blackboard that must
            # outlive a tick: a behaviour that dispatched before its terminal
            # arrived reads it on a later tick, and a step retried inside the
            # same tree still has its own goal id to match.
            clearing_policy=py_trees.common.ClearingPolicy.NEVER,
            # Deep, RELIABLE, volatile -- see `QUEUE_DEPTH`. RELIABLE matches
            # both producers; volatile because a terminal's own re-send is what
            # catches a late subscriber up, not the middleware's history.
            qos_profile=rclpy.qos.QoSProfile(
                history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
                depth=QUEUE_DEPTH,
                durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
                reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE),
        )
        self.blackboard_variable = blackboard_variable
        self.ttl = float(ttl)
        # {uuid hex: (status, producer stamp as (sec, nanosec), arrival)}.
        # Written from the subscription callback and read in `update`, both
        # under the handler's own `data_guard`. The stamp is kept only to tell
        # a re-send (same status, same stamp) from a second terminal.
        self.terminals = {}
        # {uuid hex: when it was pruned}. A producer whose TTL outlasts ours --
        # its clock runs slower than this node's, say -- is still re-sending an
        # entry we have dropped, and must not merge it back in as if new. Kept
        # for one TTL after the prune.
        self.pruned = {}
        # Goal ids a second terminal has already been reported for.
        self.conflicts = set()
        # When `update` last ran, on `_now`'s clock; None before the first.
        self.last_tick = None
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=blackboard_variable,
                                     access=py_trees.common.Access.WRITE)
        self.blackboard.set(blackboard_variable, {})

    def _now(self):
        """This node's clock, in seconds -- the one clock arrivals and ticks share.

        The node's, not the producer's: under `use_sim_time` it is the same
        `/clock` the tree's own tick timer runs on. The monotonic clock when
        there is no node yet, which only a test without one ever sees.
        """
        node = self.node
        if node is not None:
            return node.get_clock().now().nanoseconds * 1e-9
        return time.monotonic()

    def _callback(self, msg):
        """Merge one producer's array in. First write wins; never replaces."""
        now = self._now()
        conflicts = []
        with self.data_guard:
            self.msg = msg
            for item in msg.status_list:
                key = uuid_hex(item.goal_info.goal_id.uuid)
                if key is None:
                    continue
                stamp = (int(item.goal_info.stamp.sec),
                         int(item.goal_info.stamp.nanosec))
                status = int(item.status)
                held = self.terminals.get(key)
                if held is None:
                    if key in self.pruned:
                        # A re-send outliving our TTL, of an entry already
                        # dropped: not news, and not to be merged back in.
                        continue
                    self.terminals[key] = (status, stamp, now)
                    continue
                if (status, stamp) == held[:2]:
                    # The same producer re-sending the same entry: the normal
                    # case, many times a second.
                    continue
                if key not in self.conflicts:
                    self.conflicts.add(key)
                    conflicts.append((key, held, status, stamp))
        for key, held, status, stamp in conflicts:
            self._say_conflict(key, held, status, stamp)

    def _say_conflict(self, key, held, status, stamp):
        """One error line for a goal id that got a second terminal."""
        message = (
            '{}: a second terminal for goal {} -- {} (producer stamp {}.{:09d}) '
            'after {} (stamp {}.{:09d}); the first stands. One goal id has one '
            'terminal, from the producer that ran it, so one of the two '
            'producers broke the contract'.format(
                self.name, key[:16],
                _STATUS_NAMES.get(status, status), stamp[0], stamp[1],
                _STATUS_NAMES.get(held[0], held[0]), held[1][0], held[1][1]))
        node = self.node
        if node is not None:
            try:
                node.get_logger().error(message)
                return
            except Exception:
                pass
        self.logger.error(message)

    def update(self):
        """Prune by arrival against the previous tick, then publish. Always SUCCESS.

        SUCCESS whether or not anything has arrived: this runs under the root
        parallel beside the other blackboard writers, and a ledger that is
        empty because nothing has finished yet is the normal state of a chain's
        first step, not a failure of the tree.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        now = self._now()
        with self.data_guard:
            previous, self.last_tick = self.last_tick, now
            if previous is not None and self.ttl > 0.0:
                for key in [key for key, entry in self.terminals.items()
                            if previous - entry[2] > self.ttl]:
                    del self.terminals[key]
                    self.conflicts.discard(key)
                    self.pruned[key] = now
                for key in [key for key, when in self.pruned.items()
                            if now - when > self.ttl]:
                    del self.pruned[key]
            # A copy, so a callback merging while a behaviour reads cannot
            # mutate the dict underneath it. Terminals are few -- one per motion
            # in the last TTL -- so the copy is cheap at any tick rate.
            snapshot = {key: entry[0] for key, entry in self.terminals.items()}
        self.blackboard.set(self.blackboard_variable, snapshot)
        self.feedback_message = '{} terminal(s)'.format(len(snapshot))
        return py_trees.common.Status.SUCCESS
