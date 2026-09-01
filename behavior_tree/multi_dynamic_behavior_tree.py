#!/usr/bin/env python3
import importlib
import operator
import sys

import py_trees
import py_trees.console as console
import py_trees_ros
import rclpy
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from py_trees_ros import exceptions
from py_trees_ros.subscribers import ToBlackboard
from riro_srvs.srv import StringGoalStatus

from behavior_tree import decorators
from behavior_tree.dynamic_behavior_tree import SplinteredReality, get_args, load_topic_list
from behavior_tree.subtrees import Grnd2Blackboard
from behavior_tree.subtrees.OverlapSequence import (
    OVERLAP_DISPATCH_ON_START_PARAM,
    OVERLAP_THRESHOLD_PARAM,
    OverlapSequence,
)
from behavior_tree.utils.parameter_utils import make_string_list
from behavior_tree.utils.validation_utils import StepValidationResult


def create_root(robot_names):
    """
    Create a basic tree and start one goal-status blackboard writer per robot.

    Args:
        robot_names ([:obj:`str`]): list of robot names that own arm clients.

    Returns:
        :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="ROOT",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False,
        ),
    )

    grnd2bb = Grnd2Blackboard.ToBlackboard(
        name="Grnd2BB",
        topic_name="symbol_grounding",
    )

    # Define goal state per robot in blackboard.
    #
    # arm_client publishes one goal-status channel per goal channel
    # ("{robot}/arm_client/{arm,gripper}/goal_status") rather than a single one,
    # because under overlap the arm and the gripper are driven independently:
    # retiring an outgoing arm motion must not take the incoming motion's hold
    # on the gripper down with it. Move.MOVE reads "{robot}/{channel}/goal_*".
    status_nodes = []
    for robot_name in robot_names:
        for goal_channel in ["arm", "gripper"]:
            status_nodes.append(
                ToBlackboard(
                    name=f"{robot_name}_{goal_channel}_Status2BB",
                    topic_name=f"{robot_name}/arm_client/{goal_channel}/goal_status",
                    topic_type=GoalStatus,
                    blackboard_variables={
                        f"{robot_name}/{goal_channel}/goal_id": "goal_info.goal_id.uuid",
                        f"{robot_name}/{goal_channel}/goal_status": "status",
                    },
                    qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
                )
            )

        # Single-arm policy goals -> single_policy_client publishes here. It
        # writes the SAME keys as the primitive arm channel, because a policy
        # step and a primitive step are both "the arm is doing something" as far
        # as the tree is concerned, and MOVEBYPOLICY is a Move.MOVE on the arm
        # channel. Move.MOVE filters by goal_id, so the idle publisher's stale
        # id is ignored and whichever source owns the current goal wins.
        status_nodes.append(
            ToBlackboard(
                name=f"{robot_name}_PolicyStatus2BB",
                topic_name=f"{robot_name}/single_policy_client/goal_status",
                topic_type=GoalStatus,
                blackboard_variables={
                    f"{robot_name}/arm/goal_id": "goal_info.goal_id.uuid",
                    f"{robot_name}/arm/goal_status": "status",
                },
                qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
            )
        )

        # Policy progress, on its own key rather than one of arm_client's slots.
        # `step / max_steps` from the policy executor: bounded and monotonic,
        # which is all the overlap trigger needs. Separate because arm_client
        # owns the slot keys, and a primitive and a policy are live on the same
        # arm exactly when the overlap is doing its job.
        status_nodes.append(
            ToBlackboard(
                name=f"{robot_name}_PolicyProgress2BB",
                topic_name=f"{robot_name}/policy/stream/b/progress",
                topic_type=Float32,
                blackboard_variables={
                    f"{robot_name}/policy/progress": "data",
                },
                qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
            )
        )

        # Under overlap, arm motions stream through two mixer slots (a, b), and
        # each slot reports its own goal id, status, and progress. The MOVE
        # behaviours find themselves by matching their goal id against a slot.
        # These mirror the arm channel above, one pair of nodes per slot, plus a
        # progress feed the overlap composite watches to decide when to start
        # the next motion.
        for slot in ["a", "b"]:
            prefix = f"{robot_name}/arm_client/stream/{slot}"
            status_nodes.append(
                ToBlackboard(
                    name=f"{robot_name}_arm_{slot}_Status2BB",
                    topic_name=f"{prefix}/goal_status",
                    topic_type=GoalStatus,
                    blackboard_variables={
                        f"{robot_name}/arm/{slot}/goal_id": "goal_info.goal_id.uuid",
                        f"{robot_name}/arm/{slot}/goal_status": "status",
                    },
                    qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
                )
            )
            status_nodes.append(
                ToBlackboard(
                    name=f"{robot_name}_arm_{slot}_Progress2BB",
                    topic_name=f"{prefix}/progress",
                    topic_type=Float32,
                    blackboard_variables={
                        f"{robot_name}/arm/{slot}/progress": "data",
                    },
                    qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
                )
            )

    # Single dual-arm policy node publishes one goal-status channel
    # ("dual_arm_client/goal_status"), read into the shared "dual/goal_id" /
    # "dual/goal_status" keys consumed by PolicyDual.MOVEBYPOLICYDUAL. Not
    # per-robot: one 16-dim policy owns both arms, so there is one goal.
    dual_status_node = ToBlackboard(
        name="dual_Status2BB",
        topic_name="dual_arm_client/goal_status",
        topic_type=GoalStatus,
        blackboard_variables={
            "dual/goal_id": "goal_info.goal_id.uuid",
            "dual/goal_status": "status",
        },
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
    )

    priorities = py_trees.composites.Selector("Priorities", memory=False)
    priorities.add_child(py_trees.behaviours.Running(name="Idle"))

    root.add_children([grnd2bb] + status_nodes + [dual_status_node, priorities])
    return root


class MultiSplinteredReality(SplinteredReality):
    """
    Dynamic behavior tree node for coordinating multiple robot clients.
    """

    def __init__(
        self,
        jobs,
        rec_topic_list=None,
        n_loop=1,
        enable_inf_loop=False,
        loop_timeout=-1,
    ):
        """
        Initialise a multi-robot tree and preload job classes ready to be used
        when a grounding request arrives.

        Args:
            jobs ([:obj:`str`]): list of module names as strings.
            rec_topic_list (:obj:`str`): optional topics to record.
            n_loop (:obj:`int`): number of times to loop the task.
            enable_inf_loop (:obj:`bool`): whether to loop the task forever.
            loop_timeout (:obj:`float`): timeout for loop execution.
        """
        Node.__init__(
            self,
            "tree",
            automatically_declare_parameters_from_overrides=True,
        )

        # Set up ros parameters
        defaults = {
            "robot": ["left_arm", "right_arm"],
            "pose_srv_channel": "/get_object_pose",
            "grasp_pose_srv_channel": "/get_object_grasp_pose",
            "height_srv_channel": "/get_object_height",
            "rnd_pose_srv_channel": "/get_object_rnd_pose",
            "close_pose_srv_channel": "/get_object_close_pose",
            "world_frame": "world",
            # See OverlapSequence: dispatch the next motion when the previous
            # one starts moving, instead of when it reaches the threshold.
            # Declared here so it always exists for `ros2 param set`, whether or
            # not a launch file overrides it.
            "overlap_dispatch_on_start": False,
        }
        for name, value in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, value)

        # Store robot names and validate them
        self.robot_names = make_string_list(self.get_parameter("robot").value)
        self.validate_robot_names()

        # Single source of truth for where overlap starts: this node's
        # `overlap_progress_threshold` (owned by the tree, read live by
        # OverlapSequence). Each arm mixer needs the same value on its own
        # `progress_threshold`, but a node cannot read another node's
        # parameters, so the tree pushes it to the mixers whenever it changes
        # (see _sync_mixer_threshold, called each tick). Set /tree
        # overlap_progress_threshold and the mixers follow -- no separate set.
        self._mixer_param_clients = {
            robot: self.create_client(
                SetParameters, f"/{robot}/overlap_mixer/set_parameters")
            for robot in self.robot_names
        }
        self._last_pushed_threshold = None

        self.rec_topic_list = rec_topic_list
        self.n_loop = n_loop
        self.enable_inf_loop = enable_inf_loop
        self.loop_timeout = loop_timeout

        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key="edges", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="stop_cmd", access=py_trees.common.Access.WRITE)
        self.blackboard.edges = None
        self.blackboard.stop_cmd = False

        self.tree = py_trees_ros.trees.BehaviourTree(
            root=create_root(self.robot_names),
            unicode_tree_debug=True,
        )
        self.tree.add_pre_tick_handler(self.pre_tick_handler)
        self.tree.add_post_tick_handler(self.post_tick_handler)
        self.initComms()

        self.jobs = []
        for job in jobs:
            module_name = ".".join(job.split(".")[:-1])
            class_name = job.split(".")[-1]
            self.jobs.append(
                getattr(
                    importlib.import_module("behavior_tree." + module_name),
                    class_name,
                )(self)
            )
        self.current_job = None
        console.loginfo(
            f"multi_dynamic_behavior_tree: initialized for {', '.join(self.robot_names)}"
        )

    def initComms(self):
        """
        Initialize communications for each robot client and shared TF buffer.
        """
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Initialize action client per robot.
        self.action_clients = {}
        for robot_name in self.robot_names:
            service_name = f"{robot_name}/arm_client/command"
            client = self.create_client(
                StringGoalStatus,
                service_name,
                qos_profile=qos_profile,
            )
            if not client.wait_for_service(timeout_sec=3.0):
                raise exceptions.TimedOutError(
                    f"{service_name} service not available, waiting again..."
                )
            self.action_clients[robot_name] = client

        # Single dual-arm policy client. A dual_policy_execute step triggers ONE
        # dual policy node (not the per-arm arm_client clients above) through
        # this shared "dual_arm_client/command" service.
        self.dual_action_client = self.create_client(
            StringGoalStatus,
            "dual_arm_client/command",
            qos_profile=qos_profile,
        )

        # Central policy manager command client. ALL policy-related jobs
        # (single, dual, parallel) dispatch through "/policy_manager/command"
        # instead of the per-arm / dual command services. PM only relays
        # commands; goal_status is still published by the routed executor
        # ITSELF on its existing "{robot}/arm_client/goal_status" /
        # "dual_arm_client/goal_status" topic, so the existing ToBlackboard
        # wiring stays unchanged.
        self.policy_action_client = self.create_client(
            StringGoalStatus,
            "/policy_manager/command",
            qos_profile=qos_profile,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            buffer=self.tf_buffer,
            node=self,
            spin_thread=True,
            qos=qos_profile,
        )

    def _sync_mixer_threshold(self):
        """Push overlap_progress_threshold onto each arm mixer's
        progress_threshold, so the tree parameter is the single source of truth.

        Only pushes on change, and only marks a value pushed once a mixer's
        service was actually ready -- so it retries harmlessly until the mixers
        come up, then goes quiet.
        """
        if not self.has_parameter("overlap_progress_threshold"):
            return
        value = float(self.get_parameter("overlap_progress_threshold").value)
        if value == self._last_pushed_threshold:
            return
        pushed = False
        for client in self._mixer_param_clients.values():
            if not client.service_is_ready():
                continue
            request = SetParameters.Request()
            request.parameters = [
                ParameterMsg(
                    name="progress_threshold",
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=value),
                )
            ]
            client.call_async(request)
            pushed = True
        if pushed:
            self._last_pushed_threshold = value

    def pre_tick_handler(self, tree):
        """
        Check if a job is running. If not, spin up a new multi-robot job
        subtree if a request has come in.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
        # Keep the arm mixers' overlap threshold in step with the tree's.
        self._sync_mixer_threshold()

        # Look for the latest pending goal stored by any job subscriber. (Goal is the accepted entire plan.)
        goal = None
        for job in self.jobs:
            if job.goal is not None:
                goal = job.goal

        # Only build a new task subtree when the tree is idle and a goal is waiting.
        if not self.busy() and goal is not None:

            # Reject the entire goal early if any step has invalid robot assignments.
            if not self.validate_goal(goal):
                for job in self.jobs:
                    job.goal = None
                return

            # Build the cancel branch that lets an incoming stop command preempt the task.
            cancel_seq = py_trees.composites.Sequence(name="Cancel", memory=True)
            is_stop_requested = py_trees.behaviours.CheckBlackboardVariableValue(
                name="Stop?",
                check=py_trees.common.ComparisonExpression(
                    variable="stop_cmd",
                    value=True,
                    operator=operator.eq,
                ),
            )
            cancel_seq.add_child(is_stop_requested)
            task_list = []

            # Convert each step into one behaviour subtree.
            for idx in range(len(goal)):
                step_idx = str(idx + 1)
                step = goal.get(step_idx)

                requested_robot_names = make_string_list(step.get("robot"))

                # A missing robot field -> assign the one robot.
                if not requested_robot_names and len(self.robot_names) == 1:
                    requested_robot_names = [self.robot_names[0]]

                job_root = None
                for job in self.jobs:
                    # Reject the step if this job does not own the goal.
                    if job.goal is None:
                        continue

                    # Case: single-robot step -> assign the one robot
                    if len(requested_robot_names) == 1:
                        job_root = job.create_root(
                            self.action_clients[requested_robot_names[0]],
                            step_idx,
                            goal=job.goal,
                            tf_buffer=self.tf_buffer,
                            rec_topic_list=self.rec_topic_list,
                            robot_name=requested_robot_names[0],
                            policy_action_client=self.policy_action_client,
                        )

                    # Case: multi-robot step -> pass a mapping of robot_name to client.
                    # The dual policy client is passed alongside for jobs that
                    # trigger a single dual node (dual_policy_job); jobs that use
                    # the per-arm clients simply absorb it via **kwargs.
                    else:
                        job_root = job.create_root(
                            {
                                robot_name: self.action_clients[robot_name]
                                for robot_name in requested_robot_names
                            },
                            step_idx,
                            goal=job.goal,
                            tf_buffer=self.tf_buffer,
                            rec_topic_list=self.rec_topic_list,
                            robot_names=requested_robot_names,
                            dual_action_client=self.dual_action_client,
                            policy_action_client=self.policy_action_client,
                        )

                    # Case: this job cannot handle the step -> try the next job.
                    if job_root is None:
                        continue

                    # Setup the subtree immediately so it is ready before insertion.
                    console.loginfo(f"{step_idx}: pre_tick_handler running to set up all subtree modules")
                    try:
                        py_trees.trees.setup(root=job_root, node=self)
                    except RuntimeError as e:
                        console.logerror(f"RuntimeError {e}")
                    except Exception as e:
                        console.logerror(f"Exception {e}")
                    console.loginfo(f"{step_idx}: pre_tick_handler finished setting up")

                    # Keep the created subtree.
                    task_list.append(job_root)
                    break

                # Reject the entire goal if no job can actually build the step subtree.
                if job_root is None:
                    console.logwarn(
                        f"{step_idx}: pre_tick_handler rejected goal because no job built a subtree"
                    )
                    for job in self.jobs:
                        job.goal = None
                    return

            # Chain all accepted step subtrees into one task sequence.
            #
            # An OverlapSequence, because THIS is the barrier that makes a
            # `primitive -> policy -> primitive` chain stop at every seam: each
            # step is its own subtree, so overlapping within a step (which
            # move_job already does) never reaches across one. Each step root
            # reports its current child's progress upward, so this can start the
            # next step while the current one is still finishing and let the
            # mixer blend them.
            #
            # Inert until `overlap_progress_threshold` drops below 1.0: at 1.0
            # it dispatches the next step only once the current one has
            # succeeded, which is exactly Sequence(memory=True).
            task = OverlapSequence(
                name="Task",
                threshold_param=OVERLAP_THRESHOLD_PARAM,
                dispatch_param=OVERLAP_DISPATCH_ON_START_PARAM,
                progress_threshold=1.0,
            )
            task.add_children(task_list)

            # Wrap the task with either a single-run selector or a loop decorator.
            if self.n_loop <= 1 and self.enable_inf_loop is False:
                run_or_cancel = py_trees.composites.Selector(
                    name="Run or Cancel?",
                    memory=False,
                )
                run_or_cancel.add_children([cancel_seq, task])
            else:
                loop = decorators.Loop(
                    child=task,
                    name="Loop",
                    n_loop=self.n_loop,
                    enable_inf_loop=self.enable_inf_loop,
                    timeout=self.loop_timeout,
                )
                run_or_cancel = py_trees.composites.Selector(
                    name="Run or Cancel?",
                    memory=False,
                )
                run_or_cancel.add_children([cancel_seq, loop])

            # Insert the new task subtree under the priorities branch.
            root = run_or_cancel
            tree.insert_subtree(root, self.priorities.id, 0)
            console.loginfo(f"{root.name}: pre_tick_handler inserted job subtree")

            # Clear consumed goals.
            for job in self.jobs:
                job.goal = None
            return

    def validate_robot_names(self):
        """
        Validate the robot names provided in the parameters

        Returns:
            [:obj:`str`]: robot names used by this behavior tree.
        """
        
        # Case: at least one robot name must be provided.
        # TODO: For now, we only consider robot(s) with name in the ros parameter. 
        if not self.robot_names:
            raise RuntimeError(
                "Invalid multi_dynamic_behavior_tree robot parameter: "
                "parameter [robot] must define at least one robot name"
            )

        # Case: robot names must have matching parameter namespaces.
        parameter_names = self.get_parameters_by_prefix("").keys()
        parameter_namespaces = {
            parameter_name.split(".", 1)[0]
            for parameter_name in parameter_names
            if "." in parameter_name
        }
        robot_names_set = set(self.robot_names)
        if robot_names_set != parameter_namespaces:
            raise RuntimeError(
                "Invalid multi_dynamic_behavior_tree robot parameter: "
                f"parameter [robot] names {sorted(robot_names_set)} "
                f"must match parameter namespaces {sorted(parameter_namespaces)}"
            )

        # Case: robot names must not be blank.
        blank_robot_names = [
            robot_name for robot_name in self.robot_names if robot_name.strip() == ""
        ]
        if blank_robot_names:
            raise RuntimeError(
                "Invalid multi_dynamic_behavior_tree robot parameter: "
                "parameter [robot] contains an empty robot name"
            )

        # Case: robot names must be unique.
        if len(set(self.robot_names)) != len(self.robot_names):
            raise RuntimeError(
                "Invalid multi_dynamic_behavior_tree robot parameter: "
                f"parameter [robot] contains duplicate robot names: {self.robot_names}"
            )
        
    def validate_goal(self, goal):
        """
        Validate that every grounding step can be routed by this tree instance.

        Args:
            goal (:obj:`dict`): full grounding plan stored by a job.

        Returns:
            :obj:`bool`: whether every step names usable robot(s).
        """
        available_robot_names = set(self.robot_names)

        for idx in range(len(goal)):
            # Check that steps are well-formed
            step_idx = str(idx + 1)
            step = goal.get(step_idx)
            if step is None:
                console.logwarn(f"{step_idx}: validate_goal rejected goal due to missing step")
                return False

            # A missing robot field is accepted when this tree is configured with one robot.
            grounding_robot_names = make_string_list(step.get("robot"))
            if not grounding_robot_names and len(self.robot_names) == 1:
                grounding_robot_names = [self.robot_names[0]]

            # Reject the goal if any step contains robot names that are not in this tree's configuration.
            if (
                not grounding_robot_names
                or not set(grounding_robot_names).issubset(available_robot_names)
            ):
                console.logwarn(f"{step_idx}: validate_goal rejected goal due to invalid assignment")
                return False
            
            # Reject the goal if any step is mal-formatted for any job.
            job_validation_result = []
            rejecting_jobs = []
            for job in self.jobs:
                result = job.validate_step(step)
                job_validation_result.append(result)
                if result == StepValidationResult.REJECT_GOAL:
                    rejecting_jobs.append(job.__class__.__module__.split(".")[-1])
            if rejecting_jobs:
                console.logwarn(
                    f"{step_idx}: validate_goal rejected goal due to job validation failure "
                    f"from {', '.join(rejecting_jobs)}"
                )
                return False
            
            # Reject the goal unless exactly one job accepts this step.
            accept_count = sum(result == StepValidationResult.ACCEPT_GOAL for result in job_validation_result)
            if accept_count != 1:
                console.logwarn(f"{step_idx}: validate_goal rejected goal because the step is accepted by {accept_count} jobs, but should be accepted by exactly one job")
                return False

        return True


def main(args=None):
    """
    Entry point for the multi-robot dynamic behavior tree executable.
    """
    args = get_args(sysargv=sys.argv)[0]

    rclpy.init()

    if args.topic_json is None or args.topic_json.find("None") >= 0:
        topic_list = None
    else:
        topic_list = load_topic_list(args.topic_json)
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    splintered_reality = MultiSplinteredReality(
        jobs=[
            "jobs.pick_job.Move",
            "jobs.place_job.Move",
            "jobs.move_job.Move",
            "jobs.gripper_job.Move",
            "jobs.policy_job.Move",
            "jobs.dual_move_job.Move",
            "jobs.dual_policy_job.Move",
            "jobs.parallel_policy_job.Move",
        ],
        rec_topic_list=topic_list,
    )
    rclpy.get_default_context().on_shutdown(splintered_reality.shutdown)
    if not splintered_reality.setup():
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)

    splintered_reality.run()
    splintered_reality.shutdown()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
