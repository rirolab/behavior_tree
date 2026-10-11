"""Build the appliance open task from standard CAC motion subtrees."""

# Use the same grounding transport and validation contract as the other jobs.
import json
import py_trees

from behavior_tree.jobs.base_job import BaseJob
from behavior_tree.subtrees import Gripper, MovePose, MoveJoint
from behavior_tree.utils.validation_utils import StepValidationResult


class Move(BaseJob):
    def __init__(self, node):
        # Load robot parameters onto the shared tree blackboard.
        super().__init__(node)
        self.init_blackboard_parameters()
        self.scene = None

    def acceptable_step(self, step):
        # One CAC owns each articulated-object operation.
        return step.get("primitive_action") == "appliance_open" and self.check_robot_count(step, num_robot_required=1)

    def validate_step(self, step):
        # Refuse missing scene identities before the tree starts moving.
        if not self.acceptable_step(step):
            return StepValidationResult.NOT_APPLICABLE
        valid = step.get("appliance") in ("fridge", "microwave")
        return StepValidationResult.ACCEPT_GOAL if valid else StepValidationResult.REJECT_GOAL

    def incoming(self, msg):
        # Accept a complete grounding plan containing an appliance step.
        if self.goal is not None:
            self._node.get_logger().error("appliance_open_job: a goal is already pending")
            return
        grounding = json.loads(msg.data)["params"]
        if any(self.acceptable_step(step) for step in grounding.values()):
            self.goal = grounding

    def create_root(self, action_client, idx="1", goal=None, robot_name=None, **kwargs):
        # Assemble an ordinary sequence; each leaf uses the standard BT lifecycle.
        if not self.acceptable_step(goal[idx]):
            return None
        # Load MuJoCo scene messages only for appliance jobs.
        from behavior_tree.subtrees import ApplianceWorldModel
        if self.scene is None:
            self.scene = ApplianceWorldModel.SceneState(self._node, kwargs["tf_buffer"])
        # Use settled contact motions; controller speed limits may extend their durations.
        step = goal[idx]
        name = step["appliance"]
        key = f"/{robot_name}/appliance_{idx}"
        root = py_trees.composites.Sequence(name=f"{idx}: {step['primitive_action']} {name}", memory=True)
        estimator = ApplianceWorldModel.APPLIANCE_POSE_ESTIMATOR(
            "Estimate appliance targets", self.scene, step, key)
        sequence = py_trees.composites.Sequence(name="Appliance motions", memory=True)

        # Approach the handle, open the door, and retreat after releasing the gripper.
        sequence.add_children([
            MoveJoint.MOVEJ("door_open_staging", action_client, self.scene.config[name]["open_staging_joints"],
                            timeout=2.0, robot_name=robot_name, wait_until_done=True),
            Gripper.GOTO("Gripper", action_client, 0.0, timeout=1.0,
                            robot_name=robot_name, enable_wait=False, wait_until_done=True),
            MovePose.MOVES("door_open_handle_approach", action_client, {"pose": key + "/door_open_handle_approach"},
                            timeout=2.0, robot_name=robot_name, filter_size=1, wait_until_done=True),
            MovePose.MOVES("door_open_handle_grasp", action_client, {"pose": key + "/door_open_handle_grasp"},
                            timeout=2.0, robot_name=robot_name, filter_size=1, wait_until_done=True),
            Gripper.GOTO("Gripper", action_client, 1.1, timeout=3.0,
                            robot_name=robot_name, enable_wait=False, wait_until_done=True),
            MovePose.MOVEPT("door_open_arc", action_client, {"pose": key + "/door_open_arc"},
                            timeout=lambda: estimator.blackboard.get(key)["door_open_arc_seconds"],
                            robot_name=robot_name, use_moving_average=False, wait_until_done=True),
            Gripper.GOTO("Gripper", action_client, 0.0, timeout=1.0,
                            robot_name=robot_name, enable_wait=False, wait_until_done=True),
            MovePose.MOVES("door_open_retreat", action_client, {"pose": key + "/door_open_retreat"},
                            timeout=2.0, robot_name=robot_name, filter_size=1, wait_until_done=True),
        ])
        if name == "fridge":
            sequence.add_children([
                MovePose.MOVES("door_open_return_staging", action_client, {"pose": key + "/door_open_return_staging"},
                                timeout=2.0, robot_name=robot_name, filter_size=1, wait_until_done=True),
            ])

        # The standard decorator cancels the running CAC child when scene feedback is lost.
        root.add_children([estimator, py_trees.decorators.EternalGuard(
            name="Check scene feedback", condition=self.scene.safe, child=sequence,
        )])
        return root
