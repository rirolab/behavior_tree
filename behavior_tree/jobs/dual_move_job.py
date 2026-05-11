import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import Gripper, MoveJoint, MoveParallel, MovePose, WorldModel
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(base_job.BaseJob):
    """
    Generic dual-arm move: each arm runs an independent pick+place chain
    (mirroring move_job.Move) wrapped under a parallel composite so both
    arms execute simultaneously.

    Grounding step schema::

        {
          "primitive_action": "move_dual",
          "robot": ["left_arm", "right_arm"],
          "left_arm":  {"object": "cube_l", "destination": "cube_l_place"},
          "right_arm": {"object": "cube_r", "destination": "cube_r_place"}
        }
    """

    def __init__(self, node):
        super(Move, self).__init__(node)
        self.init_blackboard_parameters()

    def acceptable_step(self, step):
        if step.get("primitive_action") != "move_dual":
            return False
        if not self.check_robot_count(step, num_robot_required=2):
            return False
        return True

    def validate_step(self, step):
        if not self.acceptable_step(step):
            return StepValidationResult.NOT_APPLICABLE

        robot_names = make_string_list(step.get("robot", []))
        for robot in robot_names:
            sub = step.get(robot)
            if not isinstance(sub, dict):
                return StepValidationResult.REJECT_GOAL
            if "object" not in sub or "destination" not in sub:
                return StepValidationResult.REJECT_GOAL

        return StepValidationResult.ACCEPT_GOAL

    def incoming(self, msg):
        if self.goal:
            self._node.get_logger().error(
                "dual_move_job: rejecting new goal, previous still in the pipeline"
            )
            return

        grounding = json.loads(msg.data)["params"]
        for i in range(len(grounding.keys())):
            step = grounding.get(str(i + 1))
            if step is None:
                continue
            if self.acceptable_step(step):
                self.goal = grounding
                break

    def _per_arm_chain(self, action_client, idx, step, robot_name, tf_buffer):
        """Build the same pick+place chain that move_job.Move builds, but
        scoped to one arm's action_client, robot_name, and per-arm targets.
        """
        plan_name = f"Plan{idx}_{robot_name}"
        per_arm = step[robot_name]
        obj = per_arm["object"]
        destination = per_arm["destination"]

        blackboard = py_trees.blackboard.Client(namespace=robot_name)
        blackboard.register_key(key="gripper_open_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_pos", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_open_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="gripper_close_force", access=py_trees.common.Access.READ)
        blackboard.register_key(key="init_config", access=py_trees.common.Access.READ)

        s_init = MoveJoint.MOVEJ(
            name=f"Init_{robot_name}",
            action_client=action_client,
            action_goal=blackboard.init_config,
            robot_name=robot_name,
        )

        # ---- Pick ----
        pose_est_pick = WorldModel.POSE_ESTIMATOR(
            name=plan_name,
            object_dict={"target": obj},
            tf_buffer=tf_buffer,
            robot_name=robot_name,
        )
        pick = py_trees.composites.Sequence(name=f"Pick_{robot_name}", memory=True)
        pick.add_children([
            pose_est_pick,
            MovePose.MOVEPROOT(
                name="Top1", action_client=action_client,
                action_goal={"pose": f"{plan_name}/grasp_top_pose"},
                robot_name=robot_name,
            ),
            MovePose.MOVEP(
                name="Top2", action_client=action_client,
                action_goal={"pose": f"{plan_name}/grasp_top_pose"},
                robot_name=robot_name,
            ),
            Gripper.GOTO(
                name="Open", action_client=action_client,
                action_goal=blackboard.gripper_open_pos,
                force=blackboard.gripper_open_force,
                timeout=1, robot_name=robot_name,
            ),
            MovePose.MOVEP(
                name="Approach", action_client=action_client,
                action_goal={"pose": f"{plan_name}/grasp_pose"},
                robot_name=robot_name,
            ),
            Gripper.GOTO(
                name="Close", action_client=action_client,
                action_goal=blackboard.gripper_close_pos,
                force=blackboard.gripper_close_force,
                timeout=5, robot_name=robot_name,
            ),
            MovePose.MOVEP(
                name="Top", action_client=action_client,
                action_goal={"pose": f"{plan_name}/grasp_top_pose"},
                robot_name=robot_name,
            ),
        ])

        # ---- Place ----
        pose_est_place = WorldModel.POSE_ESTIMATOR(
            name=plan_name,
            object_dict={"target": obj, "destination": destination},
            tf_buffer=tf_buffer,
            robot_name=robot_name,
        )
        place = py_trees.composites.Sequence(name=f"Place_{robot_name}", memory=True)
        place.add_children([
            pose_est_place,
            MovePose.MOVEPROOT(
                name="Top1", action_client=action_client,
                action_goal={"pose": f"{plan_name}/place_top_pose"},
                robot_name=robot_name,
            ),
            MovePose.MOVEP(
                name="Top2", action_client=action_client,
                action_goal={"pose": f"{plan_name}/place_top_pose"},
                robot_name=robot_name,
            ),
            MovePose.MOVEP(
                name="Approach", action_client=action_client,
                action_goal={"pose": f"{plan_name}/place_pose"},
                robot_name=robot_name,
            ),
            Gripper.GOTO(
                name="Open", action_client=action_client,
                action_goal=blackboard.gripper_open_pos,
                force=blackboard.gripper_open_force,
                timeout=1, robot_name=robot_name,
            ),
            MovePose.MOVEP(
                name="Top", action_client=action_client,
                action_goal={"pose": f"{plan_name}/place_top_pose"},
                robot_name=robot_name,
            ),
            s_init,
        ])

        arm_root = py_trees.composites.Sequence(name=f"Move_{robot_name}", memory=True)
        arm_root.add_children([pick, place])
        return arm_root

    def create_root(
        self,
        action_client,
        idx="1",
        goal=std_msgs.Empty(),
        robot_names=None,
        **kwargs,
    ):
        if robot_names is None:
            assert "robot_names must be provided as a parameter or argument to create_root"
        if not self.acceptable_step(goal[idx]):
            return None

        step = goal[idx]
        action_clients = action_client  # passed as {robot_name: client} for multi-robot dispatch

        children = [
            self._per_arm_chain(
                action_client=action_clients[robot_name],
                idx=idx,
                step=step,
                robot_name=robot_name,
                tf_buffer=kwargs["tf_buffer"],
            )
            for robot_name in robot_names
        ]

        return MoveParallel.MoveParallel(name=f"DualMove{idx}", children=children)
