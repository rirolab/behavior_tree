"""Ask policy_manager what a policy skill is.

Every policy skill has an ``init_pose``: the joint pose it was trained to start
from, and out of which its first inference is out of distribution.  The executor
used to reach it with a blocking 4-second FollowJointTrajectory move at the
start of the policy goal -- which is exactly the wrong place for it under
overlap, because by the time the policy owns the arm the arm has already stopped
and there is nothing left to blend into.

Moving it to the preceding primitive needs the value in the tree.  The honest
source is the container that loaded the checkpoint, and the way to it is
``policy_manager``'s ``get_skill`` service (POLICY_CONTRACT §4): policy_manager
asks every container to ``describe`` itself at startup and serves the answers
here.  This file used to read policy_manager's registry yaml instead, and before
that cac's -- both of them copies that nothing compared against the weights, and
the second of those paths had stopped existing, so every lookup returned None,
in silence, on every run.  That is why the empty case still says so once.

Soft dependency, deliberately: if policy_manager is not running, or the skill is
unknown, or its container has not described itself yet, these return None and
the caller keeps its previous behaviour (the arm's own ``init_config``).  A tree
that cannot reach the service should still run the task, one blocking pre-move
slower.

[C3] (container redesign) A v2 skill -- a manifest in the policy registry the
tree is given (`policy_registry`, `set_registry`) -- is read from its MANIFEST,
here, by `policy_runtime.registry`: the same parser the runner and
policy_manager use, so the tree's init pose, switch and digest are the file's.
The facts carry ``kind: 'v2'``, the manifest's ``digest`` and ``hash8``, its
``switch`` and ``episode_s``; a v2 skill whose manifest does not load is
``{'kind': 'v2', 'error': ...}``, never a fallback to the service. Every other
skill is asked of ``get_skill``, as before.
"""

import json
import os

_CACHE = {}

# Where to ask. Absolute, because policy_manager is not in the tree's namespace.
_DEFAULT_SERVICE = "/policy_manager/get_skill"

# How long one lookup may take. Short: this runs while the tree is being built,
# the answer is cached per skill, and the fallback is correct if slow.
_TIMEOUT = 2.0


def set_service_name(name):
    """Point the loader at a specific ``get_skill``, or back at the default."""
    reset()
    _CACHE["service"] = name


def service_name():
    """The service this loader calls."""
    return _CACHE.get("service") or _DEFAULT_SERVICE


def set_registry(path):
    """[C3] The policy registry (v2) to read manifests from; '' or None: none.
    Keeps the service and the lookup seam; forgets the cached answers."""
    _CACHE.pop("facts", None)
    _CACHE["registry"] = str(path or "")


def registry_path():
    """The policy registry this loader reads, or ''."""
    return _CACHE.get("registry") or ""


def configure_from_node(node):
    """[C3] `set_registry` from the tree node's `policy_registry` parameter,
    when it has one and it differs from what is set. Returns the path."""
    try:
        path = str(node.get_parameter("policy_registry").value or "")
    except Exception:
        return registry_path()
    if path != registry_path():
        set_registry(path)
    return path


def _policy_runtime_registry():
    """`policy_runtime.registry`: installed, else the workspace source."""
    try:
        from policy_runtime import registry
    except ImportError:
        here = os.path.dirname(os.path.abspath(__file__))
        src = os.path.abspath(os.path.join(here, *([os.pardir] * 4)))
        import sys
        pkg = os.path.join(src, "utils", "policy_runtime")
        if pkg not in sys.path:
            sys.path.insert(0, pkg)
        from policy_runtime import registry
    return registry


def v2_skill_ids():
    """[C3] Every v2 skill id of the policy registry (set), or an empty set."""
    path = registry_path()
    if not path:
        return set()
    try:
        loaded = _policy_runtime_registry().Registry.load(path, allow_v1=True)
    except Exception as exc:
        _log_once("the policy registry {} cannot be read: {}".format(path, exc))
        return set()
    return {sid for sid, e in loaded.entries.items() if e.kind == "v2"}


def manifest_facts(skill_id):
    """[C3] A v2 skill's facts from its manifest, or None (not a v2 skill).

    Read fresh from the files on every call: the tree reads a manifest when it
    builds a step, and a manifest edited between two chains is the next
    chain's (the runner checks the digest at dispatch).
    """
    path = registry_path()
    if not path or not skill_id:
        return None
    registry = _policy_runtime_registry()
    try:
        loaded = registry.Registry.load(path, allow_v1=True)
    except Exception as exc:
        _log_once("the policy registry {} cannot be read: {}".format(path, exc))
        return None
    entry = loaded.entries.get(str(skill_id))
    if entry is None or entry.kind != "v2":
        return None
    if entry.manifest is None:
        return {"id": str(skill_id), "kind": "v2", "error": entry.error,
                "manifest_path": entry.manifest_path}
    m = entry.manifest
    joints = list(m.doc["action"]["joints"])
    facts = {"id": m.skill_id, "kind": "v2", "action_joints": joints,
             "rate_hz": m.rate_hz,
             "nominal_steps": int(m.doc["timing"]["episode_steps"]),
             "episode_s": m.episode_s, "digest": m.digest, "hash8": m.hash8,
             "manifest_path": m.source, "switch": m.switch(),
             "arm": m.arm, "manifest": m}
    pose = m.doc.get("init_pose")
    if pose is not None:
        facts["init_pose"] = dict(zip(joints, [float(v) for v in pose]))
    return facts


def set_lookup(function):
    """Replace the service call with `skill_id -> dict|None`.

    The seam a test drives: the contract this file depends on is "something
    answers with a §4 skill entry", and a test that stubs the yaml it used to
    read would be pinning a file format nothing uses any more.  `None` restores
    the real client.
    """
    reset()
    _CACHE["lookup"] = function


def reset():
    """Forget the cached answers, the client and the one-time log."""
    node = _CACHE.pop("node", None)
    if node is not None:
        try:
            node.destroy_node()
        except Exception:
            pass
    _CACHE.clear()


def _client_lookup(skill_id):
    """One `get_skill` call, on a node of this module's own.

    Its own node and its own executor because the caller is usually inside a
    behaviour tree that is already being spun: calling a service on the tree's
    node and spinning it here would re-enter its executor from its own callback
    and deadlock.  Built once and kept, so the cost is one node for the life of
    the process rather than one per lookup.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from riro_srvs.srv import StringString

    if not rclpy.ok():
        raise RuntimeError("rclpy is not initialised")
    node = _CACHE.get("node")
    if node is None:
        node = rclpy.create_node("skill_registry_client")
        _CACHE["node"] = node
        _CACHE["client"] = node.create_client(StringString, service_name())
    client = _CACHE["client"]
    if not client.wait_for_service(timeout_sec=_TIMEOUT):
        raise RuntimeError("no service on {}".format(service_name()))
    future = client.call_async(StringString.Request(data=str(skill_id)))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin_until_future_complete(future, timeout_sec=_TIMEOUT)
    finally:
        executor.remove_node(node)
    if not future.done():
        raise RuntimeError("{} did not answer in {:.0f} s".format(
            service_name(), _TIMEOUT))
    payload = json.loads(future.result().data or "{}")
    if not payload.get("found"):
        raise LookupError(payload.get("reason") or "not found")
    return payload.get("skill") or {}


def skill_facts(skill_id):
    """The §4 entry for one skill, or None. Asked once per skill."""
    if not skill_id:
        return None
    skill_id = str(skill_id)
    v2 = manifest_facts(skill_id)                               # [C3]
    if v2 is not None:
        return v2
    facts = _CACHE.setdefault("facts", {})
    if skill_id in facts:
        return facts[skill_id]
    lookup = _CACHE.get("lookup") or _client_lookup
    try:
        entry = lookup(skill_id)
    except Exception as exc:
        # Absent or unreachable is not fatal: the caller falls back to the
        # arm's own init_config, which is what it did before this existed. It is
        # worth SAYING, though -- the version of this that said nothing had been
        # returning None for every skill since the merge and no test and no log
        # noticed.
        _log_once(
            "policy skill facts unavailable, so every skill's trained start "
            "pose falls back to the arm's own init_config -- [{}]: {}".format(
                service_name(), exc))
        entry = None
    facts[skill_id] = entry if isinstance(entry, dict) else None
    return facts[skill_id]


def _log_once(message):
    """Say it on the ROS log if there is one, on stderr if there is not."""
    if _CACHE.get("logged"):
        return
    _CACHE["logged"] = True
    try:
        import rclpy.logging

        rclpy.logging.get_logger("skill_registry").warning(message)
    except Exception:
        import sys

        sys.stderr.write("skill_registry: " + message + "\n")


def skill_init_gripper(skill_id, arm_dof=7):
    """The gripper value of a skill's ``init_pose``, or None.

    The other half of ``skill_init_config``, and the reason it exists: a policy
    declares the pose it was trained to start from, the arm prefix of that pose
    goes to the preceding joint move, and the gripper value used to stop here --
    so the policy began from whatever the jaws were last left at, which is the
    difference between starting a grasp holding something and starting it
    empty. Delivered as a carried column of the approach's own trajectory, so
    the jaws are there by the time the seam opens.

    Which column: the one beyond the arm's own width, at whatever width the arm
    is. A skill declaring ``arm_dof`` columns has no gripper intent and gets
    None, which the caller passes on as an omission (rule i) rather than as a
    guess. Nothing here assumes eight.

    None as well when the skill is unknown, declares no ``init_pose``, or is
    wider than one arm plus a gripper -- a dual-arm skill's columns cannot be
    split without knowing which arm is asking, exactly as in
    ``skill_init_config``.
    """
    skill = skill_facts(skill_id)
    if not isinstance(skill, dict):
        return None
    pose = skill.get("init_pose")
    joints = skill.get("action_joints") or []
    if not pose or len(joints) != arm_dof + 1:
        return None
    try:
        return float(pose[joints[arm_dof]])
    except (KeyError, TypeError, ValueError):
        # A pose that does not cover the column it declares is not a pose.
        return None


def skill_init_config(skill_id, arm_dof=7):
    """The arm-joint prefix of a skill's ``init_pose``, or None.

    ``init_pose`` arrives keyed by joint name, in the skill's own
    ``action_joints`` order -- which for a single-arm policy is the arm's joints
    followed by the gripper.  Only the arm prefix is returned: a MOVEJ's ``goal``
    is the chain the planner plans, and a gripper value inside it would be a
    column the planner does not own. The gripper travels beside the goal instead
    -- ``skill_init_gripper``, carried through the same trajectory as a
    passthrough column.

    Returns None when the skill is unknown, declares no ``init_pose``, or is
    wider than one arm plus a gripper -- a dual-arm (16-column) skill is not
    something a single-arm primitive can be pointed at, and guessing which half
    to use would move the wrong arm.
    """
    skill = skill_facts(skill_id)
    if not isinstance(skill, dict):
        return None
    pose = skill.get("init_pose")
    joints = skill.get("action_joints") or []
    if not pose or len(joints) < arm_dof:
        return None
    if len(joints) > arm_dof + 1:
        # 16-column dual skill: the first seven are the LEFT arm, and this
        # helper has no way to know which arm is asking.
        return None
    try:
        return [float(pose[name]) for name in joints[:arm_dof]]
    except (KeyError, TypeError, ValueError):
        # A pose that does not cover the arm columns is not a pose. Falling
        # back beats moving to a partially specified one.
        return None


def skill_init_pose(skill_id, arm_dof=7):
    """A skill's whole ``init_pose``, arm columns then gripper, or None.

    What `chain_spec.resolve` needs to fill in a `pose: hub` step: the values
    the CONTAINER serves, in its own `action_joints` order. It is
    ``skill_init_config`` and ``skill_init_gripper`` joined rather than a third
    reading of the registry, so a hub step and the approach that aims at it
    cannot disagree about where the policy starts.

    Seven values on a deployment whose skill declares no gripper column, eight
    where it does -- which is exactly what `resolve` reads (`len(pose) > 7`
    decides whether the hub carries a jaw target). Nothing here assumes eight,
    and nothing assumes seven either: the width is the caller's ``arm_dof``.
    """
    arm = skill_init_config(skill_id, arm_dof=arm_dof)
    if arm is None:
        return None
    gripper = skill_init_gripper(skill_id, arm_dof=arm_dof)
    return list(arm) if gripper is None else list(arm) + [gripper]
