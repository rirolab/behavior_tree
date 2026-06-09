#!/usr/bin/env python3
import copy
from datetime import datetime
import importlib
import operator
import sys

import py_trees
import py_trees.console as console
import py_trees_ros
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from py_trees_ros import exceptions
from riro_srvs.srv import StringGoalStatus

from behavior_tree import decorators
from behavior_tree.dynamic_behavior_tree import SplinteredReality, get_args, load_topic_list
from behavior_tree.subtrees import Grnd2Blackboard, PolicyPreload, Status2Blackboard
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

    # Define arm/gripper goal state per robot in blackboard.
    status_nodes = []
    for robot_name in robot_names:
        for goal_channel in ["arm", "gripper"]:
            status_nodes.append(
                Status2Blackboard.ToBlackboard(
                    name=f"{robot_name}_{goal_channel}_Status2BB",
                    topic_name=f"{robot_name}/arm_client/{goal_channel}/goal_status",
                    robot_name=robot_name,
                    goal_channel=goal_channel,
                    qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
                )
            )

    priorities = py_trees.composites.Selector("Priorities", memory=False)
    priorities.add_child(py_trees.behaviours.Running(name="Idle"))

    root.add_children([grnd2bb] + status_nodes + [priorities])
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
            "policy_preload_enabled": True,
            "policy_preload_timeout_sec": 60.0,
            "additional_parameter_roots": ["pose_presets"],
            "drb_mode": "policy" # "teleport_ring", or "policy"
        }
        for name, value in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, value)

        # Store robot names and validate them
        self.robot_names = make_string_list(self.get_parameter("robot").value)
        self.validate_robot_names()

        self.rec_topic_list = rec_topic_list
        self.n_loop = n_loop
        self.enable_inf_loop = enable_inf_loop
        self.loop_timeout = loop_timeout

        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key="edges", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="stop_cmd", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="preloaded_policies", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="bt_start_time", access=py_trees.common.Access.WRITE)
        self.blackboard.edges = None
        self.blackboard.stop_cmd = False
        self.blackboard.preloaded_policies = []
        self.blackboard.bt_start_time = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            buffer=self.tf_buffer,
            node=self,
            spin_thread=True,
            qos=qos_profile,
        )

    def pre_tick_handler(self, tree):
        """
        Check if a job is running. If not, spin up a new multi-robot job
        subtree if a request has come in.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
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

            # Read whether policy preload branches should be attached for this task.
            policy_preload_enabled = bool(self.get_parameter("policy_preload_enabled").value)

            # Collect policy preload requests only when preload mode is enabled.
            policy_requests = []
            if policy_preload_enabled:
                policy_requests = self.collect_policy_preload_requests(goal)
            self.blackboard.preloaded_policies = []
            policy_timeout_sec = float(self.get_parameter("policy_preload_timeout_sec").value)

            # Build the cancel branch that lets an incoming stop command preempt the task.
            stop_cmd = py_trees.behaviours.CheckBlackboardVariableValue(
                name="Stop?",
                check=py_trees.common.ComparisonExpression(
                    variable="stop_cmd",
                    value=True,
                    operator=operator.eq,
                ),
            )
            cancel_seq = py_trees.composites.Sequence(name="Cancel", memory=True)
            cancel_seq.add_child(stop_cmd)
            if policy_preload_enabled and policy_requests:
                # Build the cancel-time unload behaviour directly under the failure wrapper.
                cancel_seq.add_child(
                    py_trees.decorators.FailureIsSuccess(
                        name="IgnoreUnloadFailureOnCancel",
                        child=PolicyPreload.UNLOAD_POLICY_BATCH(
                            name="UnloadPolicy",
                            action_clients=self.action_clients,
                            timeout=policy_timeout_sec,
                        ),
                    )
                )

            # Convert each step into one behaviour subtree.
            task_list = []
            
            for idx in range(len(goal)):
                step_idx = str(idx + 1)
                step = goal.get(step_idx)

                # Mark policy goals before subtree creation so execution requests reuse the same step id.
                if step.get("primitive_action") == "policy_execute" or \
                step.get("implementation") == "policy":
                    step["step_idx"] = step_idx

                # Read the robot selection for this grounding step.
                requested_robot_names = make_string_list(step.get("robot"))

                # A missing robot field -> assign the one robot.
                if not requested_robot_names and len(self.robot_names) == 1:
                    requested_robot_names = [self.robot_names[0]]

                # Try each registered job until one builds the step subtree.
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
                        )

                    # Case: multi-robot step -> pass a mapping of robot_name to client.
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
                    console.logwarn(f"{step_idx}: pre_tick_handler rejected goal because no job built a subtree")
                    for job in self.jobs:
                        job.goal = None
                    return

            # Insert policy preload/unload subtrees around the original task steps.
            if policy_preload_enabled and policy_requests:
                # Insert the preload behaviour directly before the task steps.
                task_list.insert(
                    0,
                    PolicyPreload.LOAD_POLICY_BATCH(
                        name="LoadPolicy",
                        action_clients=self.action_clients,
                        policy_requests=policy_requests,
                        timeout=policy_timeout_sec,
                    ),
                )

                # Append the unload behaviour directly after the task steps.
                task_list.append(
                    py_trees.decorators.FailureIsSuccess(
                        name="IgnoreUnloadFailureOnEnd",
                        child=PolicyPreload.UNLOAD_POLICY_BATCH(
                            name="UnloadPolicy",
                            action_clients=self.action_clients,
                            timeout=policy_timeout_sec,
                        ),
                    )
                )

            # Chain all accepted step subtrees into one task sequence.
            task = py_trees.composites.Sequence(name="Task", memory=True)
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

                # Keep the original selector structure when loop mode is enabled.
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

        # Case: robot names must have matching robot parameter namespaces.
        parameter_names = self.get_parameters_by_prefix("").keys()
        additional_parameter_roots = {
            parameter_root.strip()
            for parameter_root in make_string_list(
                self.get_parameter("additional_parameter_roots").value
            )
            if parameter_root.strip()
        }
        parameter_namespaces = {
            parameter_name.split(".", 1)[0]
            for parameter_name in parameter_names
            if "." in parameter_name
            and parameter_name.split(".", 1)[0] not in additional_parameter_roots
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
    
    def collect_policy_preload_requests(self, goal):
        """
        Extract all robot-specific policy goals that should be preloaded.

        Args:
            goal (:obj:`dict`): full grounding plan stored by a job.

        Returns:
            [(:obj:`str`, :obj:`dict`)]: list of ``(robot_name, robot_goal)`` preload requests.
        """
        requests = []
        seen_policy_configs = set()

        for idx in range(len(goal)):
            # Read one grounding step from the ordered goal payload.
            step_idx = str(idx + 1)
            step = goal.get(step_idx)
            if step is None:
                continue

            # Delegate policy extraction to the unique job that owns this step.
            for job in self.jobs:
                if job.validate_step(step) != StepValidationResult.ACCEPT_GOAL:
                    continue

                # Deduplicate requests per robot and dumped policy config.
                for robot_name, robot_goal in job.make_policy_preload_requests(step, step_idx):
                    request_key = (robot_name, PolicyPreload.dump_policy_config(robot_goal))
                    if request_key not in seen_policy_configs:
                        seen_policy_configs.add(request_key)
                        requests.append((robot_name, robot_goal))
                break

        return requests

    def post_tick_handler(self, tree):
        """
        Run terminal policy cleanup if needed, then prune the finished task subtree.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
        if self.busy():
            # The running job subtree sits just before the trailing Idle child.
            job = self.priorities.children[-2]

            # Run terminal cleanup only after the inserted job subtree has finished.
            if job.status in [
                py_trees.common.Status.SUCCESS,
                py_trees.common.Status.FAILURE,
                py_trees.common.Status.INVALID,
            ]:
                if self.blackboard.preloaded_policies:
                    # Build a one-off unload subtree for policies left over after task termination.
                    cleanup_subtree = py_trees.decorators.FailureIsSuccess(
                        name="IgnoreUnloadFailurePostTick",
                        child=PolicyPreload.UNLOAD_POLICY_BATCH(
                            name="UnloadPolicy",
                            action_clients=self.action_clients,
                            timeout=float(self.get_parameter("policy_preload_timeout_sec").value),
                        ),
                    )
                    try:
                        # Tick the cleanup subtree locally until it reaches a terminal state.
                        py_trees.trees.setup(root=cleanup_subtree, node=self)
                        while rclpy.ok():
                            cleanup_subtree.tick_once()
                            if cleanup_subtree.status in [
                                py_trees.common.Status.SUCCESS,
                                py_trees.common.Status.FAILURE,
                                py_trees.common.Status.INVALID,
                            ]:
                                break
                            rclpy.spin_once(self, timeout_sec=0.05)
                    except RuntimeError as error:
                        console.logerror(f"post_tick_handler unload setup failed: {error}")
                    except Exception as error:
                        console.logerror(f"post_tick_handler unload failed: {error}")

                # Clear the shared cache before pruning to avoid stale policy state.
                self.blackboard.preloaded_policies = []

                # Remove the finished job subtree once cleanup has completed.
                console.loginfo(f"{job.name}: post_tick_handler finished [{job.status}]")
                tree.prune_subtree(job.id)
                self.current_job = None


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
            # "jobs.pick_job.Move",
            "jobs.place_job.Move",
            "jobs.move_job.Move",
            "jobs.gripper_job.Move",
            "jobs.policy_job.Move",
            "jobs.dual_policy_job.Move",
            # DRB task related new jobs
            "jobs.drb_nopolicy_dual_grasp_job.Move",
            "jobs.drb_nopolicy_pick_job.Move",
            "jobs.drb_dual_grasp_job.Move",
            "jobs.drb_pick_job.Move",
            "jobs.tests.dual_policy_goto_test_job.Move",
            "jobs.tests.real_controller_switch_test_job.Move",
            "jobs.tests.real_hil_serl_jtc_sequence_test_job.Move",
            "jobs.real_drb_dual_grasp_job.Move",
            "jobs.real_drb_pick_job.Move",
        ],
        rec_topic_list=topic_list,
    )
    rclpy.get_default_context().on_shutdown(splintered_reality.shutdown)
    if not splintered_reality.setup():
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)

    # Run the tree with the requested tick rate.
    splintered_reality.run(tick_hz=args.tick_hz)
    splintered_reality.shutdown()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
