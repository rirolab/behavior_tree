"""Read the policy skill registry the executors read.

Every policy skill carries an ``init_config``: the joint pose the policy was
trained to start from, and out of which its first inference is out of
distribution.  Today the executor reaches it with a blocking 4-second
FollowJointTrajectory move at the start of the policy goal -- which is exactly
the wrong place for it under overlap, because by the time the policy owns the
arm the arm has already stopped and there is nothing left to blend into.

Moving it to the preceding primitive needs the value in the tree, and the only
honest way to get it is from the file the executor itself reads.  Copying the
numbers into a tree parameter would give the two sides somewhere to disagree,
and the failure mode of disagreeing -- the policy starting a few centimetres off
its trained pose -- is silent: the task just fails more often.

The registry is owned by ``policy_manager``, the node that dispatches these
skills, so this is a soft dependency: if that package is not installed, or the
skill has no ``init_config``, these return None and the caller keeps its
previous behaviour.

It used to read the file out of ``complex_action_client``'s share directory,
where the registry lived before the merge. That path stopped existing and the
loader's own ``except`` swallowed the failure by design, so every lookup
returned None, in silence, on every run -- the mechanism was merged and inert.
That is why the empty case now says so once.
"""

import os

import yaml

_CACHE = {}

# Where the registry lives, unless `set_registry_path` says otherwise. A tuple,
# because "the package that owns it" is the answer and the path inside it is
# an implementation detail of that package.
_REGISTRY_PACKAGE = "policy_manager"
_REGISTRY_RELATIVE_PATH = ("config", "skill_registry.yaml")


def set_registry_path(path):
    """Point the loader at a specific file, or back at the default with None.

    For a tree that is given `skill_registry_path` as a parameter: the tree and
    policy_manager have to read the SAME file, and the way to guarantee that is
    to pass policy_manager's parameter through rather than to have each side
    find its own.
    """
    _CACHE.clear()
    _CACHE["path"] = path


def registry_path():
    """The file this loader reads, or None when it cannot be located."""
    if _CACHE.get("path"):
        return _CACHE["path"]
    try:
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory(_REGISTRY_PACKAGE),
                            *_REGISTRY_RELATIVE_PATH)
    except Exception:
        return None


def _registry():
    """The parsed registry, or {} if it cannot be read. Loaded once."""
    if "skills" in _CACHE:
        return _CACHE["skills"]
    skills = {}
    why = None
    path = registry_path()
    if path is None:
        why = "package [{}] is not installed".format(_REGISTRY_PACKAGE)
    else:
        try:
            with open(path, "r", encoding="utf-8") as handle:
                skills = (yaml.safe_load(handle) or {}).get("skills") or {}
            if not skills:
                why = "[{}] lists no skills".format(path)
        except Exception as exc:
            # Absent or unreadable is not fatal here: the caller falls back to
            # the arm's own init_config, which is what it used before this
            # existed. It is worth SAYING, though -- the version of this that
            # said nothing had been returning None for every skill since the
            # merge and no test and no log noticed.
            why = "[{}]: {}".format(path, exc)
    if why:
        _log_once("skill registry unavailable, so every skill's trained start "
                  "pose falls back to the arm's own init_config -- " + why)
    _CACHE["skills"] = skills
    return skills


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
    """The arm-joint prefix of a skill's ``init_config``, or None.

    ``init_config`` is in the skill's ``action_joints`` order, which for a
    single-arm policy is seven arm joints followed by the gripper.  Only the arm
    prefix is returned: the preceding primitive is a joint-space arm move, and
    the gripper is commanded on its own channel -- a MOVEJ handed eight values
    would be commanding a grasp it knows nothing about.

    Returns None when the skill is unknown, has no ``init_config``, or has fewer
    values than the arm has joints -- a dual-arm (16-dim) skill is not something
    a single-arm primitive can be pointed at, and guessing which half to use
    would move the wrong arm.
    """
    skill = _registry().get(skill_id)
    if not isinstance(skill, dict):
        return None
    values = skill.get("init_config")
    if not values or len(values) < arm_dof:
        return None
    action_joints = skill.get("action_joints") or []
    if len(action_joints) > arm_dof + 1:
        # 16-dim dual skill: the first seven values are the LEFT arm, and this
        # helper has no way to know which arm is asking.
        return None
    return [float(v) for v in values[:arm_dof]]
