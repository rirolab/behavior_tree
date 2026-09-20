import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
from . import Move

from riro_srvs.srv import StringGoalStatus

class GOTO(Move.MOVE):
    """
    Move a gripper to the desired configuration

    The one gripper behaviour, on every wiring. Which writer carries the
    position out -- a gripper action server of its own, or the arm controller's
    column that the mixer passes through -- is resolved by
    ``ArmClient._gripper_writer`` and nowhere else, so this command names no
    channel and this behaviour reads whichever channel reports its goal id. A
    second behaviour per wiring, which is what ``GOTO_VIA_ARM`` was, makes the
    choice of subtree a deployment decision taken in the tree.

    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """
    def __init__(self, name, action_client, action_goal=None, force=1., check_contact=False, timeout=5, robot_name=None,
                 claim=None, enable_wait=True):
        """
        Args:
            enable_wait: whether cac's service call returns only once the jaws
                have arrived. True is what every job here has always sent, and
                it is right for a job that has nothing else to do meanwhile.
                A CHAIN passes False: the whole point of a chain is that the
                next step is dispatched while this one runs, and a call that
                does not return until the jaws are done holds the plan gate
                open for the length of the jaw motion -- so the successor's
                plan starts after this step instead of during it, and the seam
                out of a gripper step can never be anything but sequential.
                The terminal is read off the status channel either way, which
                is what this behaviour already does.
            claim: this step's own request fields, or None -- see
                ``Move.MOVE``. A gripper step of a chain file states a
                threshold like every other step (1.0: the seam nothing may
                overlap INTO -- or, on a RELEASE, a fraction of the motion
                before it, which is also what makes this step overlappable),
                and on a wiring where the jaws are a column of
                the arm controller the motion goes out through a slot and is
                claimed there like any other. Sent with the command rather than
                derived here: which writer carries it out is the client's
                answer, and a claim the client has no slot for is ignored by
                it, not by this behaviour.
        """
        super(GOTO, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal,
                                   robot_name=robot_name,
                                   goal_channel=None,
                                   claim=claim)

        self.force         = force
        self.check_contact = check_contact
        self.timeout       = timeout
        self.enable_wait   = bool(enable_wait)

        # A gripper's own motion is its own step, never overlapped *into* --
        # unless this step's own claim says otherwise.
        #
        # Read by MoveBlend of the NEXT child (`MoveBlend.py:375`), so this
        # forbids starting THIS step early -- closing before the arm has
        # reached the grasp, opening before it has reached the place. It does
        # not forbid the seam out of it: that one is decided by this step's
        # `current_progress()` against the next step's threshold, and held by
        # the mixer until the jaws are still.
        #
        # False is still the answer for every step that claims the whole of
        # its predecessor (threshold 1.0, and for a step with no claim at all).
        # A RELEASE may claim less (`chain_spec` allows a threshold in (0, 1)
        # there and refuses it on a close), and a claim nothing dispatches is
        # a number in a file that does nothing: with this False the chain goes
        # strictly sequential INTO the step and waits for the predecessor to
        # SUCCEED, so the mixer never has the release armed to open a seam
        # with. Dispatching it early is the tree's half of GRIPPER_INTO; the
        # mixer still decides where the seam actually opens, from this same
        # number.
        #
        # Explicit, since `Move.py` derives the default from the stream slots
        # and this behaviour has them: on a deployment whose gripper is a
        # passthrough column the motion really does go out through a slot.
        claimed = (self.claim or {}).get('claim_progress_threshold')
        self.overlappable = claimed is not None and float(claimed) < 1.0

    def make_command(self, uuid=None, enable_wait=None):
        """Export this gripper move as a complex action client command.

        Built here rather than inline in ``update`` so that what this
        behaviour asks the stack to do can be read without ticking it, which is
        how every other motion in this package is pinned.

        NOT through ``Move._make_command``: this command names no channel and
        no robot, and has never carried either -- the writer is the client's
        answer (``ArmClient._gripper_writer``) and adding fields to a request
        that has worked on every wiring would be a change nobody asked for.
        """
        command = {'action_type': 'gripperGotoPos',
                   'goal': self.action_goal,
                   'force': self.force,
                   'check_contact': self.check_contact,
                   'timeout': self.requested_timeout(),
                   'enable_wait': (self.enable_wait if enable_wait is None
                                   else bool(enable_wait))}
        if uuid is not None:
            command['uuid'] = uuid
        # Last, so this step's own statement wins. cac forwards the fields it
        # recognises to whichever writer carries the motion out
        # (`_action_functions.GRIPPER_CLAIM_KWARGS`) and ignores them on a
        # wiring with no slot to claim.
        if self.claim:
            command.update(self.claim)
        return command

    def ready_for_successor(self, threshold, dispatch_on_start=False):
        """May the NEXT motion be dispatched now? -- at this step's START.

        A gripper step is short: about one plan round trip (ROBUST1D §C floors
        a jaw move at 1.00 s and a plan takes 0.2-0.44 s). The inherited answer
        is `progress > 0`, which is the tick the MIXER starts the jaws moving,
        so the successor's plan begins with a fraction of a second left and
        whether it is armed before the jaws stop is a race -- the same file
        opening a seam in one run and not the next.

        So the question is answered at this step's own start instead: once this
        motion holds a slot it has been planned, armed and handed to the mixer,
        and its successor's plan round trip runs against the WHOLE jaw motion
        rather than what is left of it. Nothing about the seam moves here --
        the mixer still decides it, from the successor's claim and the veto
        that holds it until the jaws are still (rule iii). This decides only
        when the successor is planned, which is the harness's own rule (it
        waits for the previous step to reach a slot).

        Off the streaming path -- a gripper with an action server of its own --
        no slot ever carries this goal, and the chain then waits for SUCCESS,
        which is what a motion the mixer never sees has to do.
        """
        if self.status == py_trees.common.Status.SUCCESS:
            return True
        if not dispatch_on_start:
            return super(GOTO, self).ready_for_successor(
                threshold, dispatch_on_start)
        return self._owning_slot() is not None

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
                self.make_command(uuid=self.goal_uuid_des.tolist()))
            req = StringGoalStatus.Request(data=cmd_str)
            self.future = self.cmd_req.call_async(req)
            
            self.sent_goal = True
            self.feedback_message = "Sending a gripper goal"
            return py_trees.common.Status.RUNNING

        # Handle complex action client rejection before waiting for a goal-status topic.
        command_status = self.command_response_status()
        if command_status is not None:
            return command_status

        if self.current_goal_id() is None:
            return py_trees.common.Status.RUNNING
            
        if self.goal_matches_blackboard() and \
           self.current_goal_status() in [
                                GoalStatus.STATUS_UNKNOWN,
                                ]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                   self.status, \
                                   py_trees.common.Status.FAILURE, \
                                  self.feedback_message))
            return py_trees.common.Status.FAILURE

        if self.goal_matches_blackboard() and \
           self.current_goal_status() in [GoalStatus.STATUS_ABORTED,
                                               GoalStatus.STATUS_SUCCEEDED,
                                               GoalStatus.STATUS_CANCELING,
                                               GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % \
                                  (self.__class__.__name__, \
                                       self.status, \
                                  py_trees.common.Status.SUCCESS, \
                                  self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
