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
"""

import json

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


def skill_init_config(skill_id, arm_dof=7):
    """The arm-joint prefix of a skill's ``init_pose``, or None.

    ``init_pose`` arrives keyed by joint name, in the skill's own
    ``action_joints`` order -- which for a single-arm policy is seven arm joints
    followed by the gripper.  Only the arm prefix is returned: the preceding
    primitive is a joint-space arm move, and the gripper is commanded on its own
    channel -- a MOVEJ handed eight values would be commanding a grasp it knows
    nothing about.

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
