"""Run a chain file -- the same file, and the same requests, as the harness.

A chain file is a task written out step by step (`chain_spec`, and
CHAIN_FILE_BRIEF for the format). It has one other driver:
`docs/overlap/harness/merge_chain.py`, which is where the recorded seam numbers
came from. This job exists so that the behaviour tree runs the file the driver
runs, and so that the two can be compared line for line rather than by
argument:

  * ONE loader. The file is parsed, validated and resolved by the `chain_spec`
    package, which both drivers import -- so a file that runs here runs there,
    and a file that is refused is refused with the same sentence.
  * ONE pacing. `chain_spec.pacing` computes each step's duration from how far
    it travels and how fast the file asks it to go; a job that timed steps its
    own way would plan a different T and every effective-progress number would
    differ even where the thresholds matched.
  * ONE printout. `describe` is printed BEFORE the subtree is inserted and is
    byte-equal to the driver's, so the first thing in either log is the same
    table.
  * ONE result. The `RESULT` line comes from `chain_spec.run_result`.

What this job does NOT do is decide the seams. Every seam in a chain file
belongs to the step that opens it -- its `threshold` travels with the motion as
`claim_progress_threshold` and the mixer takes the seam from the incoming
motion's own claim. The `MoveBlend` here only decides when to DISPATCH, which
is why it is built with `dispatch_on_start=True` and no live threshold
parameter: a `ros2 param set /tree overlap_progress_threshold` must not
silently retune a run whose thresholds are written down in a file.

Nothing here knows the robot. The joint limits and the jaw width a file is
validated against come from `bt_config`, the hub poses from the containers'
own `init_pose`, and the gripper column travels only when the file names a
value. A chain file for a robot with no policy container and no gripper builds
and runs.

Reference: `subtrees/MoveJoint.py`, `subtrees/Policy.py`, `subtrees/Gripper.py`,
`subtrees/MoveBlend.py`, `utils/skill_registry_utils.py`.
"""

import copy
import json
import os
import sys
import time

import geometry_msgs.msg
import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.arm_trace import ArmTrace
from behavior_tree.utils import skill_registry_utils
from behavior_tree.utils.skill_registry_utils import skill_facts, skill_init_pose
from behavior_tree.utils.validation_utils import StepValidationResult
from behavior_tree.subtrees import Gripper, MoveJoint, MovePose, Policy
from behavior_tree.subtrees.MoveBlend import MoveBlend


def _import_chain_spec():
    """The `chain_spec` package: installed first, else the source tree.

    The same two-step import the harness makes, for the same reason -- the unit
    gate runs against the checkout rather than an install, and a package found
    only when someone remembered to `colcon build` is a shared loader in name
    only.
    """
    try:
        import chain_spec                                      # noqa: F401
    except ImportError:
        here = os.path.dirname(os.path.abspath(__file__))
        # .../src/behaviors/behavior_tree/behavior_tree/jobs
        src = os.path.abspath(os.path.join(here, *([os.pardir] * 4)))
        pkg = os.path.join(src, "utils", "chain_spec")
        if pkg not in sys.path:
            sys.path.insert(0, pkg)
        import chain_spec                                      # noqa: F401
    return chain_spec


chain_spec = _import_chain_spec()


class ChainJobError(RuntimeError):
    """A chain this deployment cannot run as written.

    `task` and `steps` carry what the file said about itself where the refusal
    happened late enough to know it. A refusal is a run that did nothing, and
    it is reported as one -- `RESULT task=<task> steps=0/<steps>` -- so the
    refusal ends the run the way a failure does instead of leaving the driver
    waiting for a line that is never coming.
    """

    def __init__(self, message, task=None, steps=0):
        super(ChainJobError, self).__init__(message)
        self.task = task
        self.steps = steps


def _task_of(path):
    """A refused file's task name when the file never parsed far enough to say.

    The basename without its suffix, which is what a chain file is called by
    everywhere else it is named, so the `RESULT` line of a refusal still points
    at a file rather than at `None`.
    """
    return os.path.splitext(os.path.basename(str(path or "")))[0] or "unknown"


# The file's `composition` and `defer` are the harness's knobs: it SETS the
# mixer's parameters from them before a run. The tree does not -- a tree that
# reconfigured the mixer out of a task file would change how every other
# producer on that arm is composed, for the rest of the session. It reads the
# running mixer instead and refuses a file that asks for something else, which
# is the same protection without the reach.
COMPOSITION_PARAM = "mixer_composition"

# The run's dispatch rule, and the run's alone.
#
# True (the default, and what every recorded tree run so far did): step N+1 is
# dispatched the moment step N reaches a slot, so its plan is made and armed
# while N still moves and the mixer starts it at N's claim. False: step N+1 is
# dispatched only after step N's TERMINAL, so every seam pays the tick + plan +
# arm dwell -- which is the sequential baseline the paper defines, "activates
# the successor after the predecessor completes".
#
# A node parameter, like `mixer_composition` and unlike the per-arm pacing: one
# chain is one composite, and the two dispatch rules are two different runs of
# the same file, never two arms of one run. A chain FILE may not set it
# (`load_chain`), for the same reason the mixer's composition is not a file's:
# a baseline that a task file could switch is a baseline nobody can name.
DISPATCH_PARAM = "chain_dispatch_on_start"
POLICY_REGISTRY_PARAM = "policy_registry"                 # [C3]

# Refuse a chain file whose step after an early-retreating policy states no
# threshold (REAL_RUN_FOLLOWUPS #5, `chain_spec.load(require_retreat_threshold)`).
# An arm config key read like the pacing ones, never declared: unset is off,
# and a deployment that names nothing runs exactly as before -- with the
# warning line `describe` prints and the one `create_root` logs.
REQUIRE_RETREAT_PARAM = "chain_require_retreat_threshold"

# The per-arm parameters the pacing is read from, and what each is.
#
# A chain file states each step's `speed`; the other three numbers are the
# run's, not the file's, and the harness passes them on its command line
# (`--timeout`, `--gripper-speed`, `--min-step`). The defaults below are that
# parser's, so a deployment that sets none of them paces exactly as the driver
# does with no flags -- and a run that sets them feeds BOTH drivers from one
# place rather than two.
#
# `timeout_scale` is the arm client's own: `moveJoint` divides the requested
# timeout by it, so the number in the request is wall-clock seconds TIMES the
# scale. It is read here rather than assumed, since a tree that guessed 0.5 on
# a robot configured for 1.0 would plan every step at half its length.
PACING_PARAMS = (("chain_timeout", 4.0),
                 ("chain_speed", None),
                 ("chain_gripper_speed", 0.4),
                 ("chain_min_step", 1.0))
TIMEOUT_SCALE_PARAM = "timeout_scale"
DEFAULT_TIMEOUT_SCALE = 0.5


def _claim_for(step):
    """The request fields one resolved step carries, or None.

    Exactly what `merge_chain.send_joint` puts in its payload: the step's own
    seam, and the span a carried gripper column moves over. `plan_from_arm` and
    the end-handover threshold are added by the builder for the step after a
    policy, which is the one case where the file's own threshold is not what
    the motion claims.
    """
    claim = {}
    if step.get("threshold") is not None:
        claim["claim_progress_threshold"] = float(step["threshold"])
    # Only beside a gripper target: a ramp is the span a CARRIED column moves
    # over, and a step carrying none has nothing to shape. `chain_spec` already
    # refuses the pairing, so this is the same rule twice on purpose -- the
    # payload is what cac refuses, and it should not be able to carry one.
    if step.get("gripper") is not None:
        if step.get("ramp") is not None:
            claim["passthrough_ramp"] = float(step["ramp"])
        elif step.get("ramp_fraction") is not None:
            claim["passthrough_ramp_fraction"] = float(step["ramp_fraction"])
    # The out-seam this step GRANTS the next motion, where the file said one.
    # `threshold` above is what a step claims of its PREDECESSOR; this is the
    # other direction, and only a gripper step has it -- the mixer holds the
    # seam out of a gripper step until the jaws are still (rule iii), and on a
    # RELEASE that wait is the arm standing still through the whole opening.
    # `chain_spec` refuses it anywhere else and on anything but a release, so
    # by the time a step reaches here the value is already the file's answer.
    if step.get("out_at") is not None:
        claim["claim_out_at"] = float(step["out_at"])
    return claim or None


def _is_v2(entry):
    """[C3] Is this skill-facts entry a v2 manifest (`kind: 'v2'`)?"""
    return isinstance(entry, dict) and entry.get("kind") == "v2"


class PlanFromPolicy(object):
    """[C3] The late-bound `plan_from_policy` of the step after a v2 policy.

    Read when that step's request is BUILT (`Move._make_command`): the policy
    goal's id exists only once the policy was sent, and the successor is
    dispatched while it runs, so the id is there by then. None (left out of
    the request) if the policy has not been sent -- the successor then plans
    as it would after any motion.
    """

    def __init__(self, policy, at_s, skill_id, digest):
        self.policy = policy
        self.at_s = float(at_s)
        self.skill_id = skill_id
        self.digest = digest

    def __call__(self):
        goal = getattr(self.policy, "goal_uuid_des", None)
        if goal is None:
            return None
        return {"goal": [int(b) for b in list(goal)], "at_s": self.at_s,
                "skill": self.skill_id, "manifest": self.digest}


def _v2_switch(entry, step):
    """[C3] A v2 policy step's switch: the manifest's, or the chain's
    `switch_at_s` on that step (`Manifest.switch` checks it is inside the
    episode).

    [C3c] Only from the manifest itself (`manifest_facts`, the tree's
    `policy_registry`). Facts policy_manager's `get_skill` served for a v2
    skill are not one -- a tree started without the registry -- and used to
    fail here as `'str' object has no attribute 'switch'` (ROUND36 Opus
    deviation 1); refused now, saying what to set."""
    manifest = entry.get("manifest")
    if not hasattr(manifest, "switch"):
        raise ValueError(
            "[{}] is a v2 skill and the tree has no manifest for it: its facts "
            "came from policy_manager's get_skill, which a v2 switch cannot be "
            "built from. Give the tree the stack's policy registry "
            "(`policy_registry`)".format(entry.get("id")))
    return manifest.switch(at_s=step.get("switch_at_s"))


def _claim_after_policy(claim, step, previous, facts, policy=None):
    """`_claim_for`, plus what the step AFTER a policy carries.

    Its seam is not the file's own threshold but the POLICY's:
    `end_handover.progress_threshold` is where that policy said its next motion
    may open, and `plan_from_arm` says this motion is meant to start under it
    -- planned from the measured arm now, because a policy's endpoint is not
    knowable in advance. Under the delta composition that origin cancels out of
    the endpoint (ROUND15 §7).

    ... unless THIS step states one. The container's number is the right answer
    for a step that asked for nothing, and only then: a file that names a
    threshold here has said what it wants, and `threshold_explicit` is what
    tells that apart from a value inherited from `defaults` (they can be the
    same number). `plan_from_arm` is about WHERE this motion starts, not about
    its claim, so it holds either way.

    Shared by the joint retreat and the straight one rather than written twice:
    the rule is about the POLICY before the step, and says nothing about how
    the step that follows moves.
    """
    if previous is None or previous["kind"] != "policy":
        return claim
    entry = facts.get(previous["skill_id"])
    if _is_v2(entry):
        # [C3] Time-based switching (container PLAN §5). The seam opens at the
        # manifest's `at_s` (or the chain's `switch_at_s`) as a fraction of
        # the policy's episode; a threshold the file writes here is not read
        # (warned at build, refused from C5). The plan starts where the
        # policy's window will be then (`plan_from_policy`), not at the arm.
        if not previous.get("early_retreat") or not step.get(
                "early_retreat", True):
            return claim
        switch = _v2_switch(entry, previous)
        claim = dict(claim or {})
        claim["claim_progress_threshold"] = float(switch["progress"])
        claim.pop("plan_from_arm", None)
        claim["plan_from_policy"] = PlanFromPolicy(
            policy, switch["at_s"], previous["skill_id"], entry.get("digest"))
        return claim
    handover = (entry or {}).get("end_handover") or {}
    threshold = handover.get("progress_threshold")
    if step.get("threshold_explicit") and step.get("threshold") is not None:
        threshold = float(step["threshold"])
    if threshold is not None and step.get("early_retreat", True):
        claim = dict(claim or {})
        claim["claim_progress_threshold"] = float(threshold)
        claim["plan_from_arm"] = True
    return claim


def _inline_pose(values):
    """The seven numbers a straight step states -> the `Pose` MOVES carries.

    Inline rather than through the blackboard: a chain file's pose is written
    down and has no producer to read it from. `behavior_to_pose_goal` already
    takes a `Pose` object for exactly this case.
    """
    pose = geometry_msgs.msg.Pose()
    pose.position.x = float(values[0])
    pose.position.y = float(values[1])
    pose.position.z = float(values[2])
    pose.orientation.x = float(values[3])
    pose.orientation.y = float(values[4])
    pose.orientation.z = float(values[5])
    pose.orientation.w = float(values[6])
    return pose


def chain_timeouts(resolved, args, scale, origin, gripper_origin, trace=None):
    """What each step's request asks for as its `timeout`, in the file's order.

    ONE pacing, which is the whole reason `chain_spec.pacing` is a package
    rather than a function in a driver: `schedule_with_speeds` decides each
    step's WALL-CLOCK duration from how far it travels and how fast the file
    asks it to go, and the number that goes in the request is that duration
    times `timeout_scale`, since the arm client divides by it on the way in
    (`Pacer.requested`). Both drivers compute it from the same inputs, so both
    plan the same T and every claim describes the same closure.

    A step that FOLLOWS a policy is not paced here: a policy ends where it ends,
    so the distance is unknown until that step is dispatched. Those get a
    callable, which reads the arm at dispatch and answers then -- the same
    reading `chain_spec.Pacer` makes at the same moment. Without a `trace` to
    read there is nothing to measure and they fall back to the fixed timeout,
    said out loud by the caller rather than silently.

    A policy step gets None: it takes as long as its episode takes, and its
    own timeout is the skill's.

    THE UNIT, and it is not the same on both paths. `moveJoint` divides the
    requested timeout by `timeout_scale` (trajectory_manager does it, once), so
    an arm step asks for its wall-clock duration TIMES the scale. A GRIPPER step
    does not go through a planner at all: `gripperGotoPosViaArm` builds the
    trajectory itself and spans exactly the seconds it is handed
    (`_action_functions.py`). Scaling that one would make the jaws move at half
    their length -- measured: 0.984 s against the other driver's 1.825 s for the
    same jaw travel of the same file, because that driver sends its gripper
    steps as `moveJoint` and never meets this path. So the conversion happens
    per step, here, where both units are in view, and nowhere else.
    """
    timings = chain_spec.schedule_with_speeds(
        resolved, origin, gripper_origin, args, scale)
    out = []
    for step, (duration, _reason, measured) in zip(resolved, timings):
        if step["kind"] == "straight":
            # Not paced from a distance at all: the file states the wall-clock
            # span of the line, and `chain_spec.pacing` has nothing to measure
            # -- it measures joint travel, and this step names no joint value.
            # Times the scale for the same reason a prim's is: `movePoseStraight`
            # and `moveJoint` share `_timeout_from_command`, which divides by
            # `timeout_scale` once, so both ask in the same unit.
            #
            # The step AFTER a run of these is measured at dispatch, which
            # falls out of `schedule_with_speeds` treating a non-prim as a step
            # whose endpoint it cannot know. That is the right answer here too:
            # where a straight line ends in JOINT space is the planner's IK
            # answer and is not in the file.
            out.append(float(step["duration"]) * scale)
        elif step["kind"] != "prim":
            out.append(None)
        elif measured:
            out.append(_measured_timeout(step, args, scale, trace))
        elif step.get("gripper_only"):
            out.append(duration)
        else:
            out.append(duration * scale)
    return out


def _stamps(child, attribute):
    """Every `attribute` a behaviour or its descendants carries, non-None.

    A step is one behaviour or a subtree of them -- a policy step is a wrapper
    around its motion -- so the step's own instants are the instants of the
    motions inside it.
    """
    try:
        nodes = list(child.iterate())
    except Exception:
        nodes = [child]
    return [value for value in (getattr(n, attribute, None) for n in nodes)
            if value is not None]


def _first_stamp(child, attribute):
    """The earliest of them: when this step STARTED doing whatever it does."""
    found = _stamps(child, attribute)
    return min(found) if found else None


def _last_stamp(child, attribute):
    """The latest of them: when the whole step was done with it."""
    found = _stamps(child, attribute)
    return max(found) if found else None


def _measured_timeout(step, args, scale, trace):
    """The dispatch-time pacing of one step that follows a policy.

    A callable, evaluated when the step is actually sent: `chain_spec.Pacer`
    does exactly this for the other driver -- read the arm, measure the
    distance from there, and hand `step_duration` the step's own speed.

    Always an arm step, so always the scaled unit: `chain_spec` refuses a
    gripper step straight after a policy (it would hold the arm at a pose
    nobody knows yet), which is the only way one could arrive here.
    """
    fixed = float(args.timeout)

    def resolve():
        try:
            return _paced()
        except Exception:
            # This runs inside a tick, at the moment the request is built. A
            # raise here would take the tree down over a pacing number, so the
            # fixed timeout stands in -- the value this step had before it was
            # paced at all.
            return fixed

    def _paced():
        here = None if trace is None else trace.current_config()
        if here is None or len(here) != len(step["target"]):
            # Nothing measured, or a pose of the wrong width: the fixed
            # timeout, which is what this step had before it was paced at all.
            # Visible in the run either way -- the plan the mixer describes
            # carries this number as its closure -- and the job already said
            # so once when it built the chain from a cold trace.
            return fixed
        grip = None if trace is None else trace.current_gripper()
        local = args
        speed = step.get("speed")
        if speed is not None and speed != args.speed:
            local = copy.copy(args)
            local.speed = speed
        distance = max(abs(a - b) for a, b in zip(here, step["target"]))
        jaw = (abs(step["gripper"] - grip)
               if step["gripper"] is not None and grip is not None else None)
        duration, _why = chain_spec.step_duration(distance, jaw, local, scale)
        return duration * scale

    return resolve


def build_children(resolved, action_client, policy_action_client,
                   robot_name=None, timeout=4.0, facts=None, timeouts=None):
    """The behaviours for one resolved chain, in the file's order.

    `facts` maps skill id -> the container's §4 entry, so the step AFTER a
    policy can carry that policy's `end_handover`. Passed in rather than looked
    up here, because the caller has already asked for it to resolve the hubs
    and asking twice would be two answers to one question.

    `timeouts` is `chain_timeouts` for this chain, one entry per step: the
    file's pacing. None leaves every step at the job's fixed `timeout`, which
    is what a caller with no deployment to read from gets.
    """
    facts = facts or {}
    children = []
    previous = None
    policy = None          # [C3] the MOVEBYPOLICY of the last policy step
    for index, step in enumerate(resolved):
        # The step's own paced timeout, or the fixed one where nothing paced
        # it. A policy's timeout is its episode's and is not paced here.
        paced = None if timeouts is None else timeouts[index]
        step_timeout = timeout if paced is None else paced
        if step["kind"] == "policy":
            skill_id = step["skill_id"]
            entry = facts.get(skill_id)
            handover = (entry or {}).get("end_handover")
            goal = {"skill_id": skill_id, "timeout": timeout}
            switch = None
            if _is_v2(entry):
                # [C3] The manifest edition the tree built with (the runner
                # refuses another), the chain's switch override, and the
                # switch the dispatch rule reads.
                goal["manifest"] = entry.get("digest")
                if step.get("switch_at_s") is not None:
                    goal["switch_at_s"] = float(step["switch_at_s"])
                switch = _v2_switch(entry, step)
                handover = None
            # The claim this step states, in the REQUEST. A policy's seam used
            # to be the container's `claim_progress_threshold` whatever the file
            # said: the file's number was read, printed by `describe`, and then
            # dropped on the way to the mixer, so a chain asking for 1.0 -- the
            # no-overlap baseline -- still opened a 0.98 seam. The claim belongs
            # to the request because a container serves many chains and a node
            # parameter mutated around a run belongs to none of them.
            if step.get("threshold") is not None:
                goal["claim_progress_threshold"] = float(step["threshold"])
            child = Policy.create_subtree(
                policy_action_client,
                goal,
                robot_name=robot_name,
                end_handover=handover,
                early_retreat=bool(step.get("early_retreat")),
                switch=switch,
            )
            child.name = step["label"]
            policy = child.children[0] if child.children else None
        elif step["gripper_only"]:
            # The jaws' own step: their own client, their own barrier. Never
            # overlapped INTO -- `Gripper.GOTO` says so itself -- and the seam
            # OUT of it is the NEXT step's threshold, which the file may lower.
            #
            # It carries its claim like every other step. On a wiring where the
            # jaws are a column of the arm controller the motion goes out
            # through a slot, and a step that arrived there with no claim was
            # described by the mixer's node default (0.70) instead of the
            # file's 1.0 -- the seam the file says nothing may overlap into.
            #
            # `enable_wait=False`, as the other driver sends every step: a call
            # that returns only once the jaws have arrived holds the plan gate
            # for the length of the jaw motion, so the successor's plan would
            # start after this step rather than during it.
            child = Gripper.GOTO(
                name=step["label"], action_client=action_client,
                action_goal=step["gripper"], timeout=step_timeout,
                robot_name=robot_name, claim=_claim_for(step),
                enable_wait=False)
        elif step["kind"] == "straight":
            # A Cartesian line for the tool. Everything around the goal is a
            # prim's: the same claim, the same carried jaw column, the same
            # after-a-policy rule, the same slot -- `movePoseStraight` leaves
            # through `_dispatch_arm_trajectory` exactly as `moveJoint` does,
            # kind-agnostic, so a straight step rides the mixer with its claim
            # like any other motion. What differs is the goal and its pacing.
            child = MovePose.MOVES(
                name=step["label"], action_client=action_client,
                action_goal={"pose": _inline_pose(step["pose"])},
                # The same omission rule: no field at all where the file names
                # no target, which is what keeps a grip across the step.
                gripper=step.get("gripper"),
                # The file's, and a chain file's straight step says 1 -- the
                # generator's own default of 15 rounds both ends of the line,
                # which is the thing this kind exists to avoid.
                filter_size=step.get("filter_size"),
                timeout=step_timeout, robot_name=robot_name,
                claim=_claim_after_policy(_claim_for(step), step, previous,
                                          facts, policy=policy))
        else:
            claim = _claim_after_policy(_claim_for(step), step, previous, facts,
                                        policy=policy)
            child = MoveJoint.MOVEJ(
                name=step["label"], action_client=action_client,
                action_goal=list(step["target"]),
                # ONLY when the file names a value. No field at all is the
                # omission rule: the sink holds an omitted column at its last
                # command and the mixer hands it to whichever source does have
                # a target, which a here-to-here target cannot say. `gripper:
                # hold` resolves to None here, and that is what keeps a grip
                # across the retreat.
                gripper=step.get("gripper"),
                timeout=step_timeout, robot_name=robot_name, claim=claim)
        children.append(child)
        previous = step
    return children


def build_chain_root(resolved, action_client, policy_action_client,
                     robot_name=None, timeout=4.0, facts=None, name="Chain",
                     timeouts=None, dispatch_on_start=True):
    """One `MoveBlend` over the whole file.

    `dispatch_on_start` is the RUN's, read off the node parameter
    `chain_dispatch_on_start` and passed in here; it is never a chain file's
    and never a live `dispatch_param`, so a written-down run cannot change
    rules halfway through. True (the default) is what the harness driver does
    -- step N+1 dispatched as soon as N has reached a slot, its plan paid while
    N still moves, the mixer starting it at the claim -- and reproduces every
    tree run recorded so far. False is the paper's sequential baseline: at
    threshold 1.0 nothing but step N's terminal admits step N+1
    (`default_ready_for_successor`), so every seam pays the tick + plan + arm
    dwell. The class default stays False, which is why this is passed
    explicitly in both directions.

    The composite's own `progress_threshold` is retired to dispatch-only: every
    seam in a chain file is the claim of the motion that opens it. It is left at
    1.0 and given no `threshold_param`, so nothing retunes a written-down run
    from the command line.
    """
    root = MoveBlend(name=name, progress_threshold=1.0,
                     dispatch_on_start=bool(dispatch_on_start),
                     robot_name=robot_name)
    root.add_children(build_children(
        resolved, action_client, policy_action_client,
        robot_name=robot_name, timeout=timeout, facts=facts,
        timeouts=timeouts))
    return root


class Move(base_job.BaseJob):
    """The job handler: one goal naming a chain file, one subtree."""

    def __init__(self, node):
        super(Move, self).__init__(node)
        self.init_blackboard_parameters()
        # The run's dispatch rule exists as a parameter even when no params
        # file names it, so `ros2 param set /tree chain_dispatch_on_start
        # false` reaches a stack that was brought up without it -- the reason
        # the entry points declare `overlap_dispatch_on_start`. The tree node
        # declares its parameters from its overrides
        # (`multi_dynamic_behavior_tree.py:211`), so a params file that states
        # it has already declared it and this is a no-op; nothing here changes
        # a value.
        try:
            if not node.has_parameter(DISPATCH_PARAM):
                node.declare_parameter(DISPATCH_PARAM, True)
        except Exception:
            # A node double with no parameter API, or a declaration that lost
            # a race with the entry point's. Either way the READ below answers
            # True when nothing is declared, which is this switch's default.
            pass
        # [C3] The v2 policy registry the tree reads manifests from
        # (`skill_registry_utils.configure_from_node`); '' is none. Declared
        # here so a stack that names none still answers for it.
        try:
            if not node.has_parameter(POLICY_REGISTRY_PARAM):
                node.declare_parameter(POLICY_REGISTRY_PARAM, "")
        except Exception:
            pass
        # What the run needs to answer for itself when it ends. Set at build
        # time, read by `post_tick`.
        self._task = None
        self._steps = 0
        self._started_at = None
        self._reported = False
        self._gripper_in_file = False
        # The file's OPEN value (`defaults.gripper_open`), or None where it
        # declares none: what the verdict's release is opening toward.
        self._gripper_open = None
        self._root = None
        self._resolved = []
        self._timeouts = None
        self._trace = None
        # One trace per arm, subscribed NOW rather than at goal time: the first
        # thing a chain needs from it is where the arm is, and a subscription
        # created in the same call that reads it has heard nothing yet.
        self._traces = {}
        for robot_name in (getattr(node, "robot_names", None) or [None]):
            trace = self.build_trace(robot_name)
            if trace is not None:
                self._traces[robot_name] = trace
        # [C3] Each arm's v2 runner says, per goal that ran, when its
        # successor actually started (`~/switch`, latched): the RESULT line's
        # `switch_at`. goal hex -> that JSON.
        self._switches = {}
        self._switch_subs = []
        for robot_name in (getattr(node, "robot_names", None) or []):
            self._subscribe_switch(robot_name)

    def _subscribe_switch(self, robot_name):
        try:
            from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                                   QoSReliabilityPolicy)
            from std_msgs.msg import String
            topic = "/{}/policy_runner/switch".format(robot_name)
            self._switch_subs.append(self._node.create_subscription(
                String, topic, self._on_switch,
                QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)))
        except Exception:
            # A node double without subscriptions: no switch is ever heard,
            # and `switch_at` says none.
            pass

    def _on_switch(self, msg):
        try:
            info = json.loads(msg.data)
            self._switches[str(info["goal"])] = info
        except Exception:
            pass

    def switch_fields(self, root):
        """`(switch_s, switch_at)` lists for the RESULT line, or `(None,
        None)` when no early-retreating v2 policy is in the chain."""
        if root is None:
            return None, None
        declared, actual = [], []
        for index, step in enumerate(self._resolved):
            if step["kind"] != "policy" or not step.get("early_retreat"):
                continue
            child = root.children[index] if index < len(root.children) else None
            behaviour = (child.children[0] if child is not None
                         and getattr(child, "children", None) else None)
            switch = getattr(behaviour, "switch", None)
            if not switch:
                continue
            declared.append(switch["at_s"])
            goal = getattr(behaviour, "goal_uuid_des", None)
            info = None if goal is None else self._switches.get(
                "".join("{:02x}".format(int(b) & 0xff) for b in list(goal)))
            actual.append(None if info is None else info.get("actual_s"))
        if not declared:
            return None, None
        return declared, actual

    # ------------------------------------------------------------------
    # Goal handling

    def acceptable_step(self, step):
        if step.get("primitive_action") != "chain":
            return False
        if not self.check_robot_count(step, num_robot_required=1):
            return False
        return True

    def validate_step(self, step):
        if not self.acceptable_step(step):
            return StepValidationResult.NOT_APPLICABLE
        if bool(step.get("chain_file")):
            return StepValidationResult.ACCEPT_GOAL
        return StepValidationResult.REJECT_GOAL

    def incoming(self, msg):
        if self.goal:
            self._node.get_logger().error(
                "chain_job: rejecting new goal, previous still in the pipeline")
            return
        grounding = json.loads(msg.data)["params"]
        for i in range(len(grounding.keys())):
            step = grounding.get(str(i + 1))
            if step is None:
                continue
            if self.acceptable_step(step):
                self.goal = grounding
                break

    # ------------------------------------------------------------------
    # The deployment's own numbers

    def limits_for(self, robot_name):
        """`(limits, width)` for `chain_spec.load`, from `bt_config`.

        Stated as three flat lists rather than one nested one because ROS 2
        parameters have no nested arrays; `position_limit` in the mixer's yaml
        is the same information in the shape that file can hold. Required: a
        chain file is checked against the bounds the deployment enforces, and
        this tree holds no table of its own.
        """
        lower = self._read_optional_config(robot_name, "joint_limit_lower")
        upper = self._read_optional_config(robot_name, "joint_limit_upper")
        if not lower or not upper:
            raise ChainJobError(
                "chain_job: {} states no joint_limit_lower/joint_limit_upper. "
                "A chain file is validated against this robot's own bounds, "
                "and there is no default -- a built-in table would check an "
                "FR3 pose against another arm's limits.".format(
                    robot_name or "the tree"))
        if len(lower) != len(upper):
            raise ChainJobError(
                "chain_job: {} states {} lower joint limits and {} upper "
                "ones".format(robot_name or "the tree", len(lower), len(upper)))
        limits = [(float(lo), float(hi)) for lo, hi in zip(lower, upper)]
        # None is a statement: this deployment passes no gripper column through
        # the mixer, and a file naming a gripper target is then refused by name
        # rather than checked against a range nothing would carry.
        width = self._read_optional_config(robot_name, "gripper_limit")
        width = tuple(float(v) for v in width) if width else None
        return limits, width

    def pacing_for(self, robot_name):
        """`(PacingArgs, timeout_scale)` for this arm.

        The four numbers `chain_spec.step_duration` reads, and the scale the
        request is expressed in. Each is optional and each default is the other
        driver's own (`merge_chain.py`'s parser), so a deployment that states
        none of them paces exactly as that driver does with no flags -- and a
        run that states them hands both drivers one set of numbers instead of
        two.
        """
        values = {}
        for key, fallback in PACING_PARAMS:
            value = self._read_optional_config(robot_name, key)
            # `chain_speed` unset is not a number but a statement -- "pace
            # every step at the fixed timeout" -- which is what `--speed`
            # unset means to the other driver, so None survives as None.
            values[key] = fallback if value is None else float(value)
        args = chain_spec.PacingArgs(
            timeout=values["chain_timeout"],
            speed=values["chain_speed"],
            gripper_speed=values["chain_gripper_speed"],
            min_step=values["chain_min_step"])
        scale = self._read_optional_config(robot_name, TIMEOUT_SCALE_PARAM)
        if scale is None:
            # The arm client's number, and a wrong guess at it paces every step
            # of the run wrong -- so it is said, not swallowed.
            self._node.get_logger().warning(
                "chain_job: {} states no {}; pacing this chain at {} , which "
                "is what the arm client's own parameter defaults to. A "
                "deployment configured for another scale plans every step at "
                "the wrong length.".format(robot_name or "the tree",
                                           TIMEOUT_SCALE_PARAM,
                                           DEFAULT_TIMEOUT_SCALE))
            scale = DEFAULT_TIMEOUT_SCALE
        scale = float(scale)
        if scale <= 0.0:
            raise ChainJobError(
                "chain_job: {} is {}; the arm client divides by it, so it must "
                "be positive.".format(TIMEOUT_SCALE_PARAM, scale))
        return args, scale

    def joints_for(self, robot_name):
        """`(chain joints, gripper joint or None)` for this arm, or `(None, None)`.

        The names the mixer composes, in the mixer's order -- the same source
        the joint limits come from. Needed to read the arm at all: a pose
        assembled out of `/joint_states` by position would be whichever arm the
        publisher happened to list first.
        """
        joints = self._read_optional_config(robot_name, "chain_joints")
        gripper = self._read_optional_config(robot_name, "gripper_joint")
        if not joints:
            return None, None
        return [str(j) for j in joints], (str(gripper) if gripper else None)

    def build_trace(self, robot_name):
        """This arm's `ArmTrace`, or None when it declares no joint names.

        The commanded gripper comes from the arm's own mixer, whose node name
        this tree already spells out to push its threshold
        (`multi_dynamic_behavior_tree`), and the measured one from the robot.
        An arm with no `chain_joints` gets none, and the run then answers
        `unknown` rather than a verdict nobody recorded.
        """
        joints, gripper_joint = self.joints_for(robot_name)
        if not joints:
            return None
        namespace = "/{}".format(robot_name) if robot_name else ""
        return ArmTrace(
            self._node, joints, gripper_joint,
            command_topic="{}/overlap_mixer/command_state".format(namespace))

    def mixer_composition(self):
        """The composition the running mixer is configured for, or None.

        Read, never set. `composition` and `defer` in a chain file are the
        harness's knobs -- it writes them onto the mixer's parameters before a
        run -- and a tree that did the same would change how every other
        producer on that arm is composed for the rest of the session.
        """
        try:
            value = self._node.get_parameter(COMPOSITION_PARAM).value
        except Exception:
            return None
        return None if value in (None, "") else str(value)

    def dispatch_on_start(self):
        """The run's dispatch rule, off `chain_dispatch_on_start`. True unset.

        Read the way `mixer_composition` is -- a node parameter, straight off
        the node -- and not through the blackboard: the per-arm keys are the
        robot's (limits, joint names, pacing), and this one is the RUN's. A
        deployment that declares nothing gets True, which is what every tree
        run recorded so far did.

        A string is accepted because a parameter set from a shell
        (`-p chain_dispatch_on_start:=false`) can arrive as one, and `bool("0")`
        is True -- the one spelling that would silently keep the dwell out of a
        run asked for with the dwell in it.
        """
        try:
            value = self._node.get_parameter(DISPATCH_PARAM).value
        except Exception:
            return True
        if value is None:
            return True
        if isinstance(value, str):
            return value.strip().lower() not in ("0", "false", "no", "off", "")
        return bool(value)

    def require_retreat_threshold(self, robot_name):
        """Whether this arm refuses a retreat that names no threshold. False
        unless its config says so (`REQUIRE_RETREAT_PARAM`).

        A string is read the way `dispatch_on_start` reads one, since a value
        set from a shell can arrive as one and `bool("false")` is True.
        """
        value = self._read_optional_config(robot_name, REQUIRE_RETREAT_PARAM)
        if value is None:
            return False
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def check_composition(self, spec):
        """Refuse a file whose `composition` is not what the mixer is running.

        Not a warning. The composition decides what a seam IS -- `delta` blends
        the incoming motion's own increments in, a cross-fade weights two
        absolute streams -- so a run whose file says one and whose mixer does
        the other is a measurement of neither.
        """
        wanted = (spec.get("defaults") or {}).get("composition")
        if wanted is None:
            return
        running = self.mixer_composition()
        if running is None:
            # Nothing to compare against: the tree cannot read another node's
            # parameters directly, and a deployment that does not mirror it
            # here gets the file's word for it, said out loud.
            self._node.get_logger().warning(
                "chain_job: the file asks for composition '{}' and this tree "
                "cannot see the mixer's ({} unset); running anyway"
                .format(wanted, COMPOSITION_PARAM))
            return
        if str(wanted) != running:
            raise ChainJobError(
                "chain_job: the file asks for composition '{}' and the mixer "
                "is running '{}'. The composition decides what a seam is, so "
                "this run would measure neither. Set the mixer's composition, "
                "or change the file.".format(wanted, running))

    # ------------------------------------------------------------------
    # Build

    def load_chain(self, path, robot_name, side="r"):
        """`(spec, resolved, facts)` for one chain file.

        The hubs resolve from the CONTAINERS' `init_pose`, through
        `skill_facts` -- the same registry answer the driver reads, never a
        pose written in a yaml beside a container that loaded something else.
        """
        limits, width = self.limits_for(robot_name)
        # [C3] The policy registry this tree reads v2 manifests from.
        skill_registry_utils.configure_from_node(self._node)
        spec = chain_spec.load(
            path, side, limits, width,
            require_retreat_threshold=self.require_retreat_threshold(
                robot_name),
            v2_skills=skill_registry_utils.v2_skill_ids())
        if "dispatch_on_start" in (spec.get("defaults") or {}):
            # One RUN, one dispatch rule. The rule itself is the run's, on the
            # node parameter `chain_dispatch_on_start` (`DISPATCH_PARAM`); what
            # a file may not do is set it, because a file that could turn the
            # dwell back on would be a second baseline appearing in a task
            # file, under a name only that file uses. The sentence below is
            # left word for word: it is what the refusal has always said, and
            # what it says of a FILE is still true.
            raise ChainJobError(
                "chain_job: `defaults.dispatch_on_start` is not a chain file's "
                "to set. A chain is dispatched early, always -- what the file "
                "decides is each step's threshold, which is the seam.")
        self.check_composition(spec)

        facts, init_poses = {}, {}
        for skill_id in chain_spec.skill_ids(spec):
            entry = skill_facts(skill_id)
            facts[skill_id] = entry if isinstance(entry, dict) else None
            if _is_v2(entry) and entry.get("error"):
                raise ChainJobError(
                    "chain_job: [{}] is a v2 skill whose manifest does not "
                    "load: {}".format(skill_id, entry["error"]),
                    task=spec.get("task"), steps=len(spec["steps"]))
            pose = skill_init_pose(skill_id, arm_dof=len(limits))
            if pose is not None:
                init_poses[skill_id] = pose
        self.check_v2_steps(spec, facts)
        try:
            resolved = chain_spec.resolve(spec, init_poses)
        except chain_spec.ChainSpecError as err:
            # The file parsed and its steps counted; what failed is resolving
            # one of them. The error is ANNOTATED and re-raised rather than
            # wrapped, so the refusal can be reported as `steps=0/<n>` rather
            # than as `0/0` -- the difference between "this eight-step chain
            # did nothing" and "there was no chain" -- without changing the
            # type a caller catches. `chain_spec` raises `ChainSpecError` and
            # that is what `load_chain` has always raised for a file it could
            # not resolve; wrapping it in a `ChainJobError` broke the callers
            # that name the type (`test_fr3_chain.py`, two cases).
            err.task = spec.get("task")
            err.steps = len(spec["steps"])
            raise
        return spec, resolved, facts

    def check_v2_steps(self, spec, facts):
        """[C3] The rules of a chain over v2 skills (container PLAN §5.5).

        Refused: a policy step with `early_retreat: true` on a skill whose
        manifest says `switch.early_retreat: false` (its switch is declared,
        not measured); a `switch_at_s` outside the episode. Warned (refused
        from C5): an explicit `threshold` on the step after an early-retreating
        v2 policy -- the seam is the manifest's `at_s`, and that threshold is
        not read. A `switch_at_s` on a v1 skill is refused: there is no
        manifest to override.
        """
        steps = spec["steps"]
        for index, step in enumerate(steps):
            if step["type"] != "policy":
                continue
            entry = facts.get(step["skill_id"])
            if not _is_v2(entry):
                if step.get("switch_at_s") is not None:
                    raise ChainJobError(
                        "chain_job: step '{}' sets switch_at_s, and [{}] is a "
                        "v1 skill with no manifest to override".format(
                            step["name"], step["skill_id"]),
                        task=spec.get("task"), steps=len(steps))
                continue
            try:
                switch = _v2_switch(entry, step)
            except Exception as err:
                raise ChainJobError(
                    "chain_job: step '{}': {}".format(step["name"], err),
                    task=spec.get("task"), steps=len(steps))
            if step.get("early_retreat") and not switch["early_retreat"]:
                raise ChainJobError(
                    "chain_job: step '{}' runs [{}] with early_retreat: true, and "
                    "its manifest says switch.early_retreat: false -- its "
                    "switch at {:.2f} s is declared, not measured. Run it with "
                    "early_retreat: false until it is.".format(
                        step["name"], step["skill_id"], switch["at_s"]),
                    task=spec.get("task"), steps=len(steps))
            following = steps[index + 1] if index + 1 < len(steps) else None
            if (step.get("early_retreat") and following is not None
                    and following.get("threshold_explicit")):
                self._node.get_logger().warning(
                    "chain_job: step '{}' follows the early-retreating v2 policy "
                    "'{}' and writes threshold: {}. The switch is the manifest's "
                    "at_s ({:.2f} s = {:.3f} of the episode{}); drop threshold: "
                    "{} (chain-migrate). Not read now, refused from C5.".format(
                        following["name"], step["name"],
                        following.get("threshold"), switch["at_s"],
                        switch["progress"],
                        ", the chain's switch_at_s" if switch["source"] == "chain"
                        else "", following.get("threshold")))

    def create_root(self, action_client, idx="1", goal=std_msgs.Empty(),
                    robot_name=None, **kwargs):
        step = goal[idx]
        if not self.acceptable_step(step):
            return None
        policy_action_client = kwargs.get("policy_action_client")
        path = step.get("chain_file")
        side = step.get("side") or ("l" if "left" in (robot_name or "") else "r")
        try:
            spec, resolved, facts = self.load_chain(path, robot_name, side)
        except (chain_spec.ChainSpecError, ChainJobError) as err:
            # A chain file that does not validate is a typing mistake, and the
            # answer to it is the sentence, not a stack trace or a tree that
            # half-runs.
            self.refuse_chain(
                "### the chain file cannot be run as written:\n###   {}"
                .format(err),
                task=getattr(err, "task", None) or _task_of(path),
                steps=getattr(err, "steps", 0))
            return None
        undescribed = [s["skill_id"] for s in resolved
                       if s["kind"] == "policy" and not facts.get(s["skill_id"])]
        if undescribed or (any(s["kind"] == "policy" for s in resolved)
                           and policy_action_client is None):
            # A policy step nothing describes. `policy_action_client is None`
            # was the whole guard before, and it could not fire on either tree:
            # the client passed in is the ARM's own command service
            # (`multi_dynamic_behavior_tree.py:545`), which cac serves whether
            # or not a policy exists, and `create_client` returns an object
            # regardless. MEASURED on the FR3 profile launched with
            # `with_policy:=false`: the tree built the `MoveByPolicy` subtree
            # and sent a policy goal into a graph with no policy_manager, where
            # it waits instead of being refused.
            #
            # What decides it is whether a CONTAINER describes the skill --
            # the same answer the hubs resolve from, and the same one
            # `merge_chain.py` requires before it will drive a file
            # (`open_skills`). Without it there is no episode length, no
            # `end_handover` and no init pose, so the step could not be built
            # correctly even if something did answer the goal.
            self.refuse_chain(
                "### the chain file cannot be run as written:\n"
                "###   {}: no container describes {} -- this deployment serves "
                "no such skill. A stack launched with `with_policy:=false` "
                "brings up no policy_manager and no container, so the goal "
                "would be sent and never answered, and the step's episode "
                "length and `end_handover` are unknown. Bring up the policy "
                "side for this skill, or take the policy step out."
                .format(path, ", ".join(undescribed) or "its policy step"),
                task=spec.get("task"), steps=len(resolved))
            return None

        # BEFORE the subtree is inserted, and byte-equal to what the driver
        # prints for the same file: the first thing in either log is the same
        # table, so the two runs can be diffed from their first line.
        #
        # The dispatch rule is written onto the HEADER line, never onto the
        # `RESULT` line the compare scripts parse -- and only when it is NOT
        # the driver's rule. A default run's table stays byte-equal to
        # `merge_chain.py`'s for the same file (the property `describe` exists
        # for, pinned by the suite); the run that no longer dispatches the way
        # the driver does is the one that has something to declare.
        dispatch_on_start = self.dispatch_on_start()
        table = chain_spec.describe(spec, resolved)
        if not dispatch_on_start:
            head, _, rest = table.partition("\n")
            table = "{}   dispatch after the terminal (sequential baseline)\n{}" \
                .format(head, rest)
            # ONE line, at build time, in the log the run is read from: the
            # reason a seam window in this run is 0.2-0.44 s wider than the
            # same file's yesterday is a dispatch rule, not a slower robot.
            self._node.get_logger().info(
                "chain_job: dispatch after the terminal "
                "(chain_dispatch_on_start=false): step N+1 is dispatched only "
                "after step N's terminal, so every seam pays its planning "
                "dwell -- the sequential baseline")
        print(table)
        sys.stdout.flush()
        # The table's warning line cannot name the number -- `chain_spec` has
        # no container -- and the number is the whole question: it is the
        # policy progress this retreat will wait for, however early the tree
        # dispatches it. So it is said once here, where `facts` is in hand.
        for index, step in enumerate(resolved):
            previous = resolved[index - 1] if index else None
            if (previous is None or previous["kind"] != "policy"
                    or not previous.get("early_retreat")
                    or step.get("threshold_explicit")
                    or _is_v2(facts.get(previous["skill_id"]))):
                continue
            handover = (facts.get(previous["skill_id"]) or {}).get(
                "end_handover") or {}
            if handover.get("progress_threshold") is None:
                continue
            self._node.get_logger().warning(
                "chain_job: '{}' follows the early-retreating policy '{}' and "
                "states no threshold of its own; its seam opens at {:.2f} of "
                "'{}' (the container's end_handover.progress_threshold), "
                "however early it is dispatched. Write `threshold:` on the "
                "step to choose it.".format(
                    step["label"], previous["label"],
                    float(handover["progress_threshold"]), previous["label"]))

        self._task = spec["task"]
        self._steps = len(resolved)
        self._started_at = time.time()
        self._reported = False
        self._gripper_in_file = any(s.get("gripper") is not None
                                    for s in resolved)
        gripper_open = (spec.get("defaults") or {}).get("gripper_open")
        self._gripper_open = (None if gripper_open is None
                              else float(gripper_open))

        # ONE pacing. The other driver plans each step from how far it travels
        # and how fast the file asks it to go; a job that timed its steps its
        # own way would plan a different T, and every claim would describe a
        # different closure for the same file.
        trace = self._traces.get(robot_name)
        args, scale = self.pacing_for(robot_name)
        origin = None if trace is None else trace.current_config()
        # A pose of the wrong width is not a pose: `zip` would truncate it
        # against the step's target and measure a distance over whichever
        # joints happened to line up.
        width = next((len(s["target"]) for s in resolved
                      if s.get("target") is not None), None)
        if origin is not None and width is not None and len(origin) != width:
            self._node.get_logger().warning(
                "chain_job: this arm reports {} joints and this file's steps "
                "name {}; pacing the first step from the file instead"
                .format(len(origin), width))
            origin = None
        if origin is None:
            # The arm's own pose is the origin of the FIRST step's distance,
            # and the other driver reads it at the same moment
            # (`merge_chain.run_chain`). Without it the first step is paced
            # from its own target -- a distance of zero, so the floor -- and
            # that is worth a line rather than a silently short first move.
            first = next((s["target"] for s in resolved
                          if s.get("target") is not None), None)
            origin = list(first) if first is not None else []
            self._node.get_logger().warning(
                "chain_job: nothing has reported this arm's joints yet, so "
                "the first step is paced from its own target. Every later "
                "step chains off the one before it and is unaffected.")
        self._timeouts = chain_timeouts(
            resolved, args, scale, origin,
            None if trace is None else trace.current_gripper(), trace=trace)
        self._trace = trace
        self._resolved = resolved

        self._root = build_chain_root(
            resolved, action_client, policy_action_client,
            robot_name=robot_name, facts=facts, name=spec["task"],
            timeouts=self._timeouts, dispatch_on_start=dispatch_on_start)
        return self._root

    # ------------------------------------------------------------------
    # Result

    def jaw_window(self, root):
        """`(start, end, released_at)` -- the TRANSPORT, or `(None, None, None)`.

        The stretch over which the jaws are asked to HOLD: from the end of the
        LAST policy in the file to the instant the commanded jaw starts opening
        for the release (`chain_spec.release_start`). A measured gripper that
        will not reach its command in there is holding something; outside it a
        gap is a jaw on its way somewhere, which is not the question.

        The end used to be the release step's DISPATCH, and that stopped meaning
        "the jaws are about to open" once a release could arm into the descent
        (GRIPPER_INTO) and a prefetching chain could dispatch a step in the same
        10 Hz tick the policy ended in: the window collapsed to under a
        millisecond, no 100 Hz sample landed in it, and 04:01 printed
        `held=unknown` for jaws that carried the cup (REAL_RUN_FOLLOWUPS #31,
        JAW_VERDICT_REPORT). The command is what says when the jaws were told to
        let go; the dispatch stays as the fallback for a trace that never shows
        them opening.

        `released_at` names the step that lets go (`chain_spec.releasing_step`):
        the first later step whose gripper is at least half-way from the grip to
        the open value. Not the first step that commands the jaws at all -- since
        GRASP_EVENT that is often the retreat TIGHTENING the grip on what the
        policy caught, and 05:23 printed `released_at=cu_above` for it. Where the
        grip or the open value is unknown, the old rule stands: the first later
        step that commands the jaws, dispatched after the policy ended.

        The open value is the file's `gripper_open`; a file that declares none is
        read at the command the policy STARTED from, when the jaws were open.

        The same window the other driver takes (`merge_chain.verdict`), through
        the same `chain_spec` functions over the same two stamps -- when the
        policy's motion ended (`Move.MOVE.finished_at`), and the commanded jaw.
        Reporting only: `post_tick` prints it and nothing reads it back.
        """
        last = next((i for i in range(len(self._resolved) - 1, -1, -1)
                     if self._resolved[i]["kind"] == "policy"), None)
        if last is None:
            # No policy, so no transport: nothing carried anything anywhere.
            return None, None, None
        policy = root.children[last]
        start = _last_stamp(policy, "finished_at") \
            or _first_stamp(policy, "dispatched_at")
        if start is None:
            return None, None, None
        commanded = ([] if self._trace is None
                     else self._trace.commanded_gripper())
        open_value = self._gripper_open
        if open_value is None:
            began = _first_stamp(policy, "dispatched_at")
            if began is not None:
                open_value = chain_spec.command_at(commanded, began)
        later = list(range(last + 1, len(self._resolved)))
        which = chain_spec.releasing_step(
            [self._resolved[i].get("gripper") for i in later],
            chain_spec.command_at(commanded, start), open_value)
        end = chain_spec.release_start(commanded, start, open_value)
        released_at = None
        if which is not None:
            release = later[which]
            sent = _first_stamp(root.children[release], "dispatched_at")
            if sent is not None:
                # Dispatched, so it is what let go -- whenever it was sent.
                released_at = self._resolved[release]["label"]
                if end is None and sent > start:
                    end = sent
            return start, end, released_at
        for index in later:
            if self._resolved[index].get("gripper") is None:
                continue
            sent = _first_stamp(root.children[index], "dispatched_at")
            if sent is not None and sent > start:
                released_at = self._resolved[index]["label"]
                if end is None:
                    end = sent
                break
        return start, end, released_at

    def jaw_verdict(self, root):
        """`(held, jaw_gap, released_at)` for this run's `RESULT` line.

        `n/a` when the file commands no gripper at all -- a robot without one
        is a valid deployment, not a failed run. `None` (printed `unknown`)
        when nothing published a column over the window, since `false` there
        would say the jaws were empty and what happened is that nobody spoke.
        The test itself is the shared one, so the two drivers' verdicts are one
        function's output rather than two opinions.
        """
        if not self._gripper_in_file:
            return "n/a", None, None
        if self._trace is None:
            return None, None, None
        start, end, released_at = self.jaw_window(root)
        if start is None:
            return None, None, None
        held, gap = chain_spec.jaws_held(
            self._trace.commanded_gripper(), self._trace.measured_gripper(),
            (start, end if end is not None else self._trace.now()))
        return held, gap, released_at

    def refuse_chain(self, message, task=None, steps=0):
        """Refuse a chain out loud AND end the run: the reason, then `RESULT`.

        Every layer under this one already answers a skill nothing serves. The
        container describes one checkpoint and refuses any other id by name;
        `policy_manager` turns that into `found: false` with the container and
        the id in the reason (`_get_skill_callback`); `skill_facts` returns
        None; and `chain_spec.resolve` or the guard above says which step of
        which file cannot be built. What no layer did was END THE RUN: the job
        returned None, `pre_tick_handler` dropped the goal, and the `RESULT`
        line -- the one thing every driver of a chain file answers with -- was
        never printed. Measured on Isaac (TREE2B5 §6c) and reproduced on a stub
        (`phase1/stub_cube_like_unserved.yaml`): no RESULT, no seam, the arm at
        park, and the run ending only on the caller's wall clock. On a robot
        that is a typo parking the arm under a silent tree.

        So a refusal reports itself as what it is -- a run in which none of the
        file's steps ran -- through the same formatter a finished chain uses,
        which is why `steps=0/{n}` and not a second kind of line. `held` is
        `unknown` rather than `n/a`: nothing was driven, so nothing was
        measured, and a refusal must not be able to look like a deployment
        without a gripper.
        """
        self._node.get_logger().error(message)
        # `_reported` guards the pair: `post_tick` prints at most one RESULT
        # per chain and so does this, and a refused chain has no root for
        # `post_tick` to read anyway.
        self._reported = True
        self._root = None
        line = chain_spec.run_result(task or "unknown", 0, int(steps or 0),
                                     None, None, None, 0.0)
        print(line)
        sys.stdout.flush()
        return line

    def post_tick(self):
        """The `RESULT` line, once, when the chain stops running.

        The driver's formatter, so the fields compare -- the jaw verdict
        included: the columns come from the deployment's own transport (the
        mixer's `~/command_state` for what was commanded, `/joint_states` for
        what the jaws did) and go through `chain_spec.jaws_held`, which is the
        test the other driver makes.
        """
        root = self._root
        if root is None or self._reported:
            return None
        if root.status not in (py_trees.common.Status.SUCCESS,
                               py_trees.common.Status.FAILURE):
            return None
        self._reported = True
        done = sum(1 for child in root.children
                   if child.status == py_trees.common.Status.SUCCESS)
        try:
            held, gap, released_at = self.jaw_verdict(root)
        except Exception as exc:
            # The verdict is one field of the line; the line itself says what
            # the run did and must be printed either way.
            self._node.get_logger().warning(
                "chain_job: the jaw verdict could not be taken [{}]".format(exc))
            held, gap, released_at = None, None, None
        switch_s, switch_at = self.switch_fields(root)
        line = chain_spec.run_result(
            self._task, done, self._steps, held, gap, released_at,
            time.time() - (self._started_at or time.time()),
            switch_s=switch_s, switch_at=switch_at)
        print(line)
        sys.stdout.flush()
        return line
