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

The registry lives in complex_action_client's share directory, so this is a
soft dependency: if cac is not installed, or the skill has no ``init_config``,
these return None and the caller keeps its previous behaviour.
"""

import os

import yaml

_CACHE = {}


def _registry():
    """The parsed registry, or {} if it cannot be read. Loaded once."""
    if "skills" in _CACHE:
        return _CACHE["skills"]
    skills = {}
    try:
        from ament_index_python.packages import get_package_share_directory

        path = os.path.join(
            get_package_share_directory("complex_action_client"),
            "config",
            "skill_registry.yaml",
        )
        with open(path, "r", encoding="utf-8") as handle:
            skills = (yaml.safe_load(handle) or {}).get("skills") or {}
    except Exception:
        # Absent or unreadable is not an error here: the caller falls back to
        # the arm's own init_config, which is what it used before this existed.
        skills = {}
    _CACHE["skills"] = skills
    return skills


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
