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
publishes ITS terminals to, re-sent while they are inside the TTL.

Two properties of this behaviour follow from there being MANY publishers, and
both are the opposite of what `ToBlackboard` does:

  * it MERGES rather than overwrites, and never clears. Each producer sends its
    own subset, so the last array to arrive is not the whole truth -- taking it
    as the whole truth would drop the arm client's terminals every time the
    policy container published, and the other way round.
  * it merges in the CALLBACK, not in `update`. Two publishers' arrays landing
    between two ticks of a 2 Hz tree is the normal case, not the corner, and
    keeping only `self.msg` would discard the first of them.

Pruning is by the stamp the PRODUCER put in `goal_info.stamp` -- when the
motion ended -- so a re-send does not refresh its own entry's age.
"""

import py_trees
import rclpy.qos

from action_msgs.msg import GoalStatusArray

from py_trees_ros import subscribers


#: Seconds an entry survives after the stamp its producer gave it. Matches the
#: producers' `terminal_ledger_ttl` default. Longer than a producer's is safe
#: (a terminal this old belongs to a step that finished long ago); shorter only
#: means an entry is dropped and then re-merged from the next re-send.
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
        # {uuid hex: (status, stamp)}. Written from the subscription callback
        # and read in `update`, both under the handler's own `data_guard`.
        self.terminals = {}
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key=blackboard_variable,
                                     access=py_trees.common.Access.WRITE)
        self.blackboard.set(blackboard_variable, {})

    def _callback(self, msg):
        """Merge one producer's array in. Never replaces the dict."""
        with self.data_guard:
            self.msg = msg
            for item in msg.status_list:
                key = uuid_hex(item.goal_info.goal_id.uuid)
                if key is None:
                    continue
                stamp = item.goal_info.stamp
                self.terminals[key] = (
                    int(item.status),
                    float(stamp.sec) + float(stamp.nanosec) * 1e-9)

    def update(self):
        """Prune by stamp, then publish the dict. Always SUCCESS.

        SUCCESS whether or not anything has arrived: this runs under the root
        parallel beside the other blackboard writers, and a ledger that is
        empty because nothing has finished yet is the normal state of a chain's
        first step, not a failure of the tree.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        now = (self.node.get_clock().now().nanoseconds * 1e-9
               if self.node is not None else None)
        with self.data_guard:
            if now is not None and self.ttl > 0.0:
                for key in [key for key, entry in self.terminals.items()
                            if now - entry[1] > self.ttl]:
                    del self.terminals[key]
            # A copy, so a callback merging while a behaviour reads cannot
            # mutate the dict underneath it. Terminals are few -- one per motion
            # in the last TTL -- so the copy is cheap at any tick rate.
            snapshot = {key: status for key, (status, _) in
                        self.terminals.items()}
        self.blackboard.set(self.blackboard_variable, snapshot)
        self.feedback_message = '{} terminal(s)'.format(len(snapshot))
        return py_trees.common.Status.SUCCESS
