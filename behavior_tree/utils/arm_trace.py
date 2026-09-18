"""What the arm and its jaws are doing, for a job that has to answer for a run.

Two questions a chain job cannot answer out of the tree alone, and both are
about the deployment rather than about the tree:

  * WHERE THE ARM IS. A step that follows a policy is planned from the measured
    arm -- a policy ends where it ends -- so its duration is not knowable until
    it is dispatched. The other driver reads the arm at that moment
    (`chain_spec.Pacer.primitive`); this is the same reading, so the two plan
    the same T.
  * WHAT THE JAWS DID. `RESULT` says whether the gripper stalled short of what
    was commanded of it, which takes both traces on one clock: the COMMANDED
    value, from the mixer's own output, and the MEASURED one, from the robot.
    Without them the line can only say `unknown`, which is a run that proves
    nothing about the grasp.

Nothing here knows a robot. The joint names arrive from the caller, which reads
them out of the mixer's configuration, and a deployment that passes no gripper
column names none -- `gripper_joint=None` -- and every jaw answer is then None,
which `chain_spec.run_result` prints as `n/a`.

The traces are bounded: one chain is under two minutes and the gripper column
arrives at the mixer's rate, so a cap of a few thousand samples is minutes of
run and a fixed ceiling on a node that stays up between goals.
"""

from collections import deque

from sensor_msgs.msg import JointState


# Two traces per run at 100 Hz: ~2 minutes each, which is longer than any chain
# in this workspace and still a fixed bound on a tree that never restarts.
TRACE_SAMPLES = 12000


class ArmTrace:
    """The arm's measured pose and the jaw column, commanded and measured."""

    def __init__(self, node, joints, gripper_joint=None,
                 states_topic="/joint_states", command_topic=None,
                 depth=TRACE_SAMPLES):
        """
        Args:
            node: the ROS node the subscriptions live on.
            joints: the chain's joint names, in the chain's order. The measured
                pose is assembled BY NAME, never by position: `/joint_states`
                carries both arms and the order is the publisher's.
            gripper_joint: the passthrough column the file's gripper targets
                move, or None on a deployment that has none.
            command_topic: where the COMMANDED gripper value is published --
                the mixer's `~/command_state`, which is what actually reached
                the controller after every producer was composed. None leaves
                the commanded trace empty, and `jaws_held` then answers None
                rather than guessing from the measured side alone.
        """
        self._node = node
        self._joints = [str(j) for j in joints]
        self._gripper_joint = str(gripper_joint) if gripper_joint else None
        self._measured = {}
        self._jaw_measured = deque(maxlen=depth)
        self._jaw_commanded = deque(maxlen=depth)
        self._subs = [node.create_subscription(
            JointState, states_topic, self._on_state, 10)]
        if command_topic:
            self._subs.append(node.create_subscription(
                JointState, command_topic, self._on_command, 10))

    # ------------------------------------------------------------------

    def _stamp(self, msg):
        """The message's own clock, falling back to this node's.

        One clock for both traces, which is what `jaws_held` compares them on.
        A publisher that stamps nothing would otherwise put the two traces
        seconds apart and the verdict would be about the clocks.
        """
        stamp = msg.header.stamp
        seconds = stamp.sec + stamp.nanosec * 1e-9
        if seconds > 0.0:
            return seconds
        return self._node.get_clock().now().nanoseconds * 1e-9

    def _on_state(self, msg):
        when = self._stamp(msg)
        for name, position in zip(msg.name, msg.position):
            self._measured[str(name)] = float(position)
            if self._gripper_joint and str(name) == self._gripper_joint:
                self._jaw_measured.append((when, float(position)))

    def _on_command(self, msg):
        if not self._gripper_joint:
            return
        when = self._stamp(msg)
        for name, position in zip(msg.name, msg.position):
            if str(name) == self._gripper_joint:
                self._jaw_commanded.append((when, float(position)))

    # ------------------------------------------------------------------

    def current_config(self):
        """The chain's joints as measured, in the chain's order, or None.

        None when any one of them has not been heard: a pose with a hole in it
        would be a distance measured against a joint that is not there, and the
        caller's answer to "I do not know where the arm is" is not a zero.
        """
        try:
            return [self._measured[name] for name in self._joints]
        except KeyError:
            return None

    def current_gripper(self):
        """The jaw column as measured, or None when this run has no column."""
        if not self._gripper_joint:
            return None
        return self._measured.get(self._gripper_joint)

    def commanded_gripper(self):
        """`[(t, value)]` for the jaw column as the mixer published it."""
        return list(self._jaw_commanded)

    def measured_gripper(self):
        """`[(t, value)]` for the jaw column as the robot reports it."""
        return list(self._jaw_measured)

    def now(self):
        """This node's clock, the one both traces are stamped against."""
        return self._node.get_clock().now().nanoseconds * 1e-9

    def destroy(self):
        for sub in self._subs:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass
        self._subs = []
