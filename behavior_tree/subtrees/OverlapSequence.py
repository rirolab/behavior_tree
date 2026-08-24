"""A sequence whose consecutive arm motions overlap instead of running strictly one at a time.

A plain ``Sequence(memory=True)`` ticks child N and refuses to touch child N+1
until N returns SUCCESS. That is what forces the arm to stop dead at every
waypoint: each motion runs to completion, then the next begins from rest.

``OverlapSequence`` keeps the same order and the same success semantics -- it
succeeds only when the last child succeeds, fails the moment any child fails --
but it starts ticking child N+1 early, as soon as child N reports its progress
has crossed ``progress_threshold``. Both children then tick each cycle: N is
still finishing while N+1 has been dispatched, so their motions are live at the
same time and the mixer blends them. When N finally succeeds, authority passes
to N+1, which may already have pulled N+2 in behind it.

Overlap only happens between children that actually expose progress -- i.e.
streamed arm motions. A gripper op or a world-model query reports no progress,
so ``current_progress()`` returns None for it, the early-start test never fires,
and the sequence falls back to strict sequential across that boundary. That is
exactly the desired safety: the grasp closes only after the arm has arrived, the
place happens only after the transit has finished.

``progress_threshold`` here is the trigger the *tree* owns -- when to dispatch
the next motion. How the two are combined once both are live (an additive
velocity sum) is owned by the mixer, downstream. Setting the threshold to 1.0
makes this behave exactly like ``Sequence(memory=True)``: the next child starts
only when the current one is fully done.

``dispatch_on_start`` splits that one trigger in two. Dispatching a motion is
not free -- a tree tick at 2 Hz plus the primitive's IK, measured at 0.2 to
0.44 s -- and under the threshold gate all of it is spent *after* the mixer
already wanted to blend. Measured on the 80 mm square, a requested 0.75 fired
at an effective 0.908, and 0.60 at 0.976, i.e. no overlap at all. With this on,
the next motion is dispatched as soon as the newest one is moving at all, so
that cost is paid while the previous motion still runs; the mixer's own
``progress_threshold`` still decides when the blend actually opens.

It is self-limiting, which is why there is no depth counter here: a dispatched
motion sits ARMED until the mixer starts it, and an ARMED slot reports progress
0.0, so the gate blocks again until the mixer moves on. At most three motions
are ever in flight -- two slots plus one queued -- which is exactly what
``dispatch_stream`` accepts before rejecting.

Off by default because it changes the *sequential* baseline too. At threshold
1.0 the mixer still runs the motions strictly one after another, but the
planning dwell between them (measured 0.204 to 0.255 s of commanded standstill
at each corner) disappears, since the next plan is built during the current
motion. Any comparison against previously recorded sequential runs has to keep
this off.
"""

import py_trees
from py_trees import common


class OverlapSequence(py_trees.composites.Composite):

    def __init__(self, name="OverlapSequence", children=None,
                 progress_threshold=1.0, threshold_param=None,
                 dispatch_on_start=False, dispatch_param=None):
        super().__init__(name=name, children=children)
        # The fraction of a child's motion that must be done before the next
        # child is dispatched. 1.0 == wait for full completion == plain
        # Sequence.
        self.progress_threshold = progress_threshold
        # When set, read the threshold live from this node parameter each tick,
        # so `ros2 param set /tree <param> <x>` retunes overlap on the fly. A
        # node parameter, not a blackboard key: the job mirrors parameters onto
        # the blackboard only once at startup, so a blackboard copy would stay
        # frozen at its initial value and ros2 param set would appear to do
        # nothing.
        self.threshold_param = threshold_param
        # Dispatch the next motion as soon as the newest one is *moving*,
        # instead of waiting for it to reach the threshold. See the module
        # docstring: this decouples "when to plan" from "when to blend", and
        # it is off by default because it changes the sequential baseline.
        self.dispatch_on_start = dispatch_on_start
        self.dispatch_param = dispatch_param
        self._node = None
        # Highest index dispatched so far. Children 0..started are all live;
        # authority (the child whose success advances the tree) is `current`.
        self._started = 0
        self._current = 0

    def setup(self, **kwargs):
        # py_trees_ros hands the ROS node in here; keep it so the threshold can
        # be read live.
        self._node = kwargs.get("node", self._node)
        return True

    def _threshold(self):
        if self.threshold_param is not None and self._node is not None:
            try:
                value = self._node.get_parameter(self.threshold_param).value
                if value is not None:
                    return float(value)
            except Exception:
                pass
        return float(self.progress_threshold)

    def _dispatch_on_start(self):
        if self.dispatch_param is not None and self._node is not None:
            try:
                value = self._node.get_parameter(self.dispatch_param).value
                if value is not None:
                    return bool(value)
            except Exception:
                pass
        return bool(self.dispatch_on_start)

    def _child_progress(self, child):
        """Progress in [0,1] for a child, or None if it exposes none."""
        getter = getattr(child, "current_progress", None)
        if getter is None:
            return None
        try:
            return getter()
        except Exception:
            return None

    def _child_overlappable(self, child):
        """Whether the NEXT child may be started early, before this-1 finishes.

        Only a streamed arm motion may: it publishes progress and its command is
        blended by the mixer. A gripper op runs on its own controller and would
        act the instant it is dispatched -- close before the arm reaches the
        grasp, open before it reaches the place -- so it must wait. A
        world-model query is not a motion at all. Both are recognised by having
        no stream slots (gripper: goal_channel != 'arm'; query: not a MOVE).
        Decided by type, not by live progress: a not-yet-dispatched arm motion
        reports None progress but is still overlappable.
        """
        return bool(getattr(child, "stream_slots", ()))

    def tick(self):
        self.logger.debug("%s.tick()" % self.__class__.__name__)

        if not self.children:
            self.current_child = None
            self.stop(common.Status.SUCCESS)
            yield self
            return

        # Fresh entry (not resuming a RUNNING tick): reset and invalidate any
        # children left non-INVALID from a previous run, matching Sequence.
        if self.status != common.Status.RUNNING:
            self._started = 0
            self._current = 0
            self.current_child = self.children[0]
            for child in self.children:
                if child.status != common.Status.INVALID:
                    child.stop(common.Status.INVALID)
            self.initialise()

        # Tick every live child, from authority to newest-started, advancing
        # authority as children succeed. A child that succeeds hands off to the
        # next, which must itself be ticked this same cycle -- otherwise it sits
        # a tick in INVALID and tip() finds no live leaf. This loop keeps going
        # until the authority child is one that did not just succeed, mirroring
        # how a plain Sequence flows straight from one success into the next.
        while True:
            newly_ticked = False
            last = self._started
            for index in range(self._current, last + 1):
                if index >= len(self.children):
                    break
                child = self.children[index]
                # SUCCESS: already handed off. FAILURE: a live overlap partner
                # failed -- do NOT re-tick it, or Move.initialise (sent_goal =
                # False) would resend it as a brand-new goal and silently revive
                # a motion that was aborting. It is handled by the failure scan
                # below instead.
                if child.status in (common.Status.SUCCESS,
                                    common.Status.FAILURE):
                    continue
                newly_ticked = True
                for node in child.tick():
                    yield node

            # Any live child failed -- not only the authority one. An overlap
            # partner (index > _current) can fail while the authority is still
            # RUNNING; the sequence must still abandon cleanly rather than let
            # the partner's stale state drive the early-start gate below.
            for index in range(self._current,
                               min(self._started, len(self.children) - 1) + 1):
                if self.children[index].status == common.Status.FAILURE:
                    failed = self.children[index]
                    self._stop_from(0, common.Status.INVALID, skip=index)
                    self.current_child = failed
                    self.stop(common.Status.FAILURE)
                    yield self
                    return

            advanced = False
            while self._current < len(self.children):
                child = self.children[self._current]
                if child.status == common.Status.SUCCESS:
                    self._current += 1
                    if self._started < self._current:
                        self._started = self._current
                    advanced = True
                    continue
                break

            # All children done?
            if self._current >= len(self.children):
                self.current_child = self.children[-1]
                self.stop(common.Status.SUCCESS)
                yield self
                return

            # Re-tick only when authority advanced onto a not-yet-ticked child,
            # so it is never left INVALID for a whole tick. Otherwise this tick
            # is done.
            authority = self.children[self._current]
            if advanced and authority.status == common.Status.INVALID:
                continue
            break

        # Should the next, not-yet-started child be dispatched now?
        #
        # Two conditions, both required:
        #  - the NEWEST started child must be far enough along (its progress >=
        #    threshold), and
        #  - the NEXT child must itself be overlappable -- it must expose
        #    progress. A gripper op or world-model query reports None, and must
        #    NOT be started early: the gripper is on its own controller and would
        #    act the instant it is dispatched, i.e. close before the arm has
        #    reached the grasp pose, or open before the arm has reached the place
        #    pose (dropping the object). So the chain goes strictly sequential
        #    into a non-overlappable child: it waits for the newest to SUCCEED.
        if self._started < len(self.children) - 1:
            newest = self.children[self._started]
            nxt = self.children[self._started + 1]
            newest_progress = self._child_progress(newest)
            if newest.status == common.Status.SUCCESS:
                newest_progress = 1.0

            if self._child_overlappable(nxt):
                if self._dispatch_on_start():
                    # Dispatch as soon as the newest motion is actually moving.
                    # Planning it costs a tree tick plus the primitive's IK, and
                    # under the threshold gate all of that lands *after* the
                    # mixer already wanted to blend, so the overlap starts late:
                    # a requested 0.75 was measured firing at an effective
                    # 0.908. Started here instead, that cost is paid while the
                    # previous motion is still running.
                    #
                    # This cannot run away. A dispatched motion sits ARMED until
                    # the mixer starts it, and an ARMED slot reports progress
                    # 0.0, so the gate blocks again immediately -- at most one
                    # motion is ever queued beyond the two live ones, which is
                    # exactly what `dispatch_stream` accepts.
                    start_next = (newest_progress is not None
                                  and newest_progress > 0.0)
                else:
                    start_next = (newest_progress is not None
                                  and newest_progress >= self._threshold())
            else:
                # Non-overlappable successor (gripper, query): only start it once
                # the newest has genuinely finished.
                start_next = newest.status == common.Status.SUCCESS

            if start_next:
                self._started += 1
                # The freshly started child gets its first tick this cycle, so
                # it dispatches without a one-tick lag.
                child = self.children[self._started]
                if child.status != common.Status.SUCCESS:
                    for node in child.tick():
                        yield node

        # Point current_child at a child that is actually RUNNING, so tip()
        # resolves to a live leaf. The authority child is the natural choice,
        # but a child that was only just started this tick, or one that has
        # already succeeded, can leave tip() empty; fall back to the newest live
        # child, then to the authority regardless.
        self.current_child = self._live_child()

        self.status = common.Status.RUNNING
        yield self

    def _live_child(self):
        """A started child that is currently RUNNING, for tip() to descend into.

        Prefer the authority child (the one whose success advances the tree);
        fall back to any other started-and-running child; last resort, the
        authority child even if not RUNNING, so current_child is never None.
        """
        span = range(self._current, min(self._started + 1, len(self.children)))
        for index in span:
            if self.children[index].status == common.Status.RUNNING:
                return self.children[index]
        return self.children[self._current]

    def _stop_from(self, start, status, skip=None):
        for index in range(start, min(self._started + 1, len(self.children))):
            if index == skip:
                continue
            child = self.children[index]
            if child.status != common.Status.INVALID:
                child.stop(status)

    def stop(self, new_status=common.Status.INVALID):
        # On INVALID (higher-priority interrupt), make sure every child we
        # started is torn down, not just the authority one.
        if new_status == common.Status.INVALID:
            self._stop_from(0, common.Status.INVALID)
            self._started = 0
            self._current = 0
        super().stop(new_status)
