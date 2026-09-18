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

import json
import os
import sys
import time

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.skill_registry_utils import skill_facts, skill_init_pose
from behavior_tree.utils.validation_utils import StepValidationResult
from behavior_tree.subtrees import Gripper, MoveJoint, Policy
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
    """A chain this deployment cannot run as written."""


# The file's `composition` and `defer` are the harness's knobs: it SETS the
# mixer's parameters from them before a run. The tree does not -- a tree that
# reconfigured the mixer out of a task file would change how every other
# producer on that arm is composed, for the rest of the session. It reads the
# running mixer instead and refuses a file that asks for something else, which
# is the same protection without the reach.
COMPOSITION_PARAM = "mixer_composition"


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
    return claim or None


def build_children(resolved, action_client, policy_action_client,
                   robot_name=None, timeout=4.0, facts=None):
    """The behaviours for one resolved chain, in the file's order.

    `facts` maps skill id -> the container's §4 entry, so the step AFTER a
    policy can carry that policy's `end_handover`. Passed in rather than looked
    up here, because the caller has already asked for it to resolve the hubs
    and asking twice would be two answers to one question.
    """
    facts = facts or {}
    children = []
    previous = None
    for step in resolved:
        if step["kind"] == "policy":
            skill_id = step["skill_id"]
            handover = (facts.get(skill_id) or {}).get("end_handover")
            child = Policy.create_subtree(
                policy_action_client,
                {"skill_id": skill_id, "timeout": timeout},
                robot_name=robot_name,
                end_handover=handover,
                early_retreat=bool(step.get("early_retreat")),
            )
            child.name = step["label"]
        elif step["gripper_only"]:
            # The jaws' own step: their own client, their own barrier. Never
            # overlapped INTO -- `Gripper.GOTO` says so itself -- and the seam
            # OUT of it is the NEXT step's threshold, which the file may lower.
            child = Gripper.GOTO(
                name=step["label"], action_client=action_client,
                action_goal=step["gripper"], timeout=timeout,
                robot_name=robot_name)
        else:
            claim = _claim_for(step)
            if previous is not None and previous["kind"] == "policy":
                # The retreat. Its seam is not the file's own threshold but the
                # POLICY's: `end_handover.progress_threshold` is where that
                # policy said its next motion may open, and `plan_from_arm`
                # says this motion is meant to start under it -- planned from
                # the measured arm now, because a policy's endpoint is not
                # knowable in advance. Under the delta composition that origin
                # cancels out of the endpoint (ROUND15 §7).
                handover = (facts.get(previous["skill_id"]) or {}).get(
                    "end_handover") or {}
                threshold = handover.get("progress_threshold")
                if threshold is not None and step.get("early_retreat", True):
                    claim = dict(claim or {})
                    claim["claim_progress_threshold"] = float(threshold)
                    claim["plan_from_arm"] = True
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
                timeout=timeout, robot_name=robot_name, claim=claim)
        children.append(child)
        previous = step
    return children


def build_chain_root(resolved, action_client, policy_action_client,
                     robot_name=None, timeout=4.0, facts=None, name="Chain"):
    """One `MoveBlend` over the whole file.

    `dispatch_on_start=True`, passed explicitly and not through a node
    parameter. The harness dispatches step N+1 as soon as N has reached a slot
    and lets the mixer start it at the claim, so it has no planning dwell; this
    is that, and a chain run with it off would be a third baseline nobody
    recorded. The class default stays False -- the old jobs' recorded
    sequential baseline is threshold 1.0 with the dwell in it.

    The composite's own `progress_threshold` is retired to dispatch-only: every
    seam in a chain file is the claim of the motion that opens it. It is left at
    1.0 and given no `threshold_param`, so nothing retunes a written-down run
    from the command line.
    """
    root = MoveBlend(name=name, progress_threshold=1.0,
                     dispatch_on_start=True, robot_name=robot_name)
    root.add_children(build_children(
        resolved, action_client, policy_action_client,
        robot_name=robot_name, timeout=timeout, facts=facts))
    return root


class Move(base_job.BaseJob):
    """The job handler: one goal naming a chain file, one subtree."""

    def __init__(self, node):
        super(Move, self).__init__(node)
        self.init_blackboard_parameters()
        # What the run needs to answer for itself when it ends. Set at build
        # time, read by `post_tick`.
        self._task = None
        self._steps = 0
        self._started_at = None
        self._reported = False
        self._gripper_in_file = False
        self._root = None

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
            self._node.get_logger().warn(
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
        spec = chain_spec.load(path, side, limits, width)
        if "dispatch_on_start" in (spec.get("defaults") or {}):
            # One driver, one dispatch rule. A file that could turn the dwell
            # back on would be a second baseline appearing in a task file.
            raise ChainJobError(
                "chain_job: `defaults.dispatch_on_start` is not a chain file's "
                "to set. A chain is dispatched early, always -- what the file "
                "decides is each step's threshold, which is the seam.")
        self.check_composition(spec)

        facts, init_poses = {}, {}
        for skill_id in chain_spec.skill_ids(spec):
            entry = skill_facts(skill_id)
            facts[skill_id] = entry if isinstance(entry, dict) else None
            pose = skill_init_pose(skill_id, arm_dof=len(limits))
            if pose is not None:
                init_poses[skill_id] = pose
        resolved = chain_spec.resolve(spec, init_poses)
        return spec, resolved, facts

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
            self._node.get_logger().error(
                "### the chain file cannot be run as written:\n###   {}"
                .format(err))
            return None
        if any(s["kind"] == "policy" for s in resolved) \
                and policy_action_client is None:
            self._node.get_logger().error(
                "chain_job: {} has policy steps and this tree was built with "
                "no policy_action_client".format(path))
            return None

        # BEFORE the subtree is inserted, and byte-equal to what the driver
        # prints for the same file: the first thing in either log is the same
        # table, so the two runs can be diffed from their first line.
        print(chain_spec.describe(spec, resolved))
        sys.stdout.flush()

        self._task = spec["task"]
        self._steps = len(resolved)
        self._started_at = time.time()
        self._reported = False
        self._gripper_in_file = any(s.get("gripper") is not None
                                    for s in resolved)
        self._root = build_chain_root(
            resolved, action_client, policy_action_client,
            robot_name=robot_name, facts=facts, name=spec["task"])
        return self._root

    # ------------------------------------------------------------------
    # Result

    def post_tick(self):
        """The `RESULT` line, once, when the chain stops running.

        The driver's formatter, so the fields compare. `held` is `n/a` when the
        file names no gripper column -- a robot without one is a valid
        deployment, not a failed run -- and `unknown` where a jaw trace exists
        but nothing published over the transport. This job holds no trace, so
        it answers `n/a` or `unknown`; a run that wants the jaw verdict records
        the columns and passes them through `chain_spec.jaws_held`.
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
        line = chain_spec.run_result(
            self._task, done, self._steps,
            None if self._gripper_in_file else "n/a", None, None,
            time.time() - (self._started_at or time.time()))
        print(line)
        sys.stdout.flush()
        return line
