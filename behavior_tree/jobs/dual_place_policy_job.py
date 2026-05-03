import json

import py_trees
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.utils.validation_utils import StepValidationResult
from . import dual_policy_common as dual


class Move(base_job.BaseJob):
    """
    Dual-arm place job that runs the same policy goal on both arms in parallel.
    """

    def __init__(self, node):
        """
        Tune into a channel for incoming goal requests. This is a simple
        subscriber here but more typically would be a service or action interface.
        """
        super(Move, self).__init__(node)
        self.init_blackboard_parameters()

    def acceptable_step(self, step):
        """
        Check whether this job should accept a grounding step for this primitive action.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :obj:`bool`: whether this job can take ownership of the step.
        """
        # Check if the primitive action is dual_place_policy
        if step.get("primitive_action") != "dual_place_policy":
            return False

        # Check if the step has the number of robots required for this job
        elif not self.check_robot_count(step, num_robot_required=2):
            return False

        else:
            return True

    def validate_step(self, step):
        """
        Validate whether an acceptable dual place policy step is well-formed
        enough to keep the overall goal.

        Args:
            step (:obj:`dict`): one grounding step from the incoming goal.

        Returns:
            :class:`StepValidationResult`: whether this step should be accepted
            for this job, rejected as malformed, or ignored as not acceptable.
        """
        # Check if the step has the required parameters for dual place policy
        if self.acceptable_step(step):
            if not any(key in step for key in ["object", "obj", "target", "held_object"]):
                return StepValidationResult.REJECT_GOAL
            elif any(key in step for key in ["destination", "dest"]):
                return StepValidationResult.ACCEPT_GOAL
            else:
                return StepValidationResult.REJECT_GOAL
        else:
            return StepValidationResult.NOT_APPLICABLE

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            self._node.get_logger().error(
                "dual_place_policy_job: rejecting new goal, previous still in the pipeline"
            )
        else:
            grounding = json.loads(msg.data)["params"]
            for i in range(len(grounding.keys())):
                step = grounding.get(str(i + 1))
                if step is None:
                    continue
                if self.acceptable_step(step):
                    self.goal = grounding
                    break

    def create_root(
        self,
        action_client,
        idx="1",
        goal=std_msgs.Empty(),
        robot_names=None,
        **kwargs,
    ):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        # Check if the step is acceptable
        if not self.acceptable_step(goal[idx]):
            return None
        command = goal[idx]

        robots = robot_names or dual.robot_names_from_command(command)
        action_clients = action_client
        timeout = float(command.get("timeout", command.get("timeout_sec", 5.0)))
        move_timeout = float(command.get("move_timeout", command.get("move_timeout_sec", 3.0)))
        straight_timeout = float(command.get("straight_timeout", move_timeout))
        plan_name = "Plan" + idx

        # ----------------- Place ---------------------
        s_init1 = dual.make_parameter_pose_parallel(
            name="DualMoveInitTogether",
            action_clients=action_clients,
            robots=robots,
            parameter_name="init_together",
            motion_type="move_straight",
            timeout=straight_timeout,
        )
        pose_est1 = dual.make_pose_estimator(
            name=plan_name,
            command=command,
            robots=robots,
            tf_buffer=kwargs["tf_buffer"],
            include_destination=True,
        )
        s_move11 = dual.make_blackboard_pose_parallel(
            name="DualMovePlaceTop",
            action_clients=action_clients,
            robots=robots,
            pose_key=plan_name + "/place_top_pose",
            motion_type="move_straight",
            timeout=straight_timeout,
        )
        s_move12 = dual.make_policy_parallel(
            name="DualPlacePolicyRun",
            action_clients=action_clients,
            robots=robots,
            command=command,
            timeout=timeout,
        )
        s_move13 = dual.make_blackboard_pose_parallel(
            name="DualMovePlaceTopAfterPolicy",
            action_clients=action_clients,
            robots=robots,
            pose_key=plan_name + "/place_top_pose",
            motion_type="move_straight",
            timeout=straight_timeout,
        )
        s_move14 = dual.make_parameter_pose_parallel(
            name="DualMoveInitPose",
            action_clients=action_clients,
            robots=robots,
            parameter_name="init_pose",
            motion_type="move_pose",
            timeout=move_timeout,
        )

        root = py_trees.composites.Sequence(name="DualPlacePolicy", memory=True)
        root.add_children([s_init1, pose_est1, s_move11, s_move12, s_move13, s_move14])
        return root
