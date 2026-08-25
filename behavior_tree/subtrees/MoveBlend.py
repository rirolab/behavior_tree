import json
import typing

import numpy as np
import py_trees
from action_msgs.msg import GoalStatus
from behavior_tree.utils.blend_status import BlendPhase
from behavior_tree.utils.blend_status import BlendProgressStatus
from riro_srvs.srv import StringGoalStatus


class MoveBlend(py_trees.composites.Composite):
    """
    Run a command sequence by sending incremental blend steps to complex action client.
    """

    def __init__(self, name, action_client, timeout=1.0, blend_duration=0.0, 
                 check_contact=False, robot_name=None, goal_channel="arm", 
                 children=None):
        # Define the complex action client blend command schema used by this composite.
        self.arm_goal_channel = "arm"
        self.blend_action_type = "blend"
        self.blend_initial_child_count = 2
        self.blend_next_child_count = 1

        # Use child motion nodes as the only source for blend commands.
        if children is None:
            children = []

        # Validate blend configuration before pretick tree validation.
        blend_duration, blend_reject_reason = self._validate_blend(
            children, blend_duration, goal_channel, robot_name,
        )

        super(MoveBlend, self).__init__(name=name, children=children)

        # Store complex action client command state for incremental blend dispatch.
        self.cmd_req = action_client
        self.timeout = timeout
        self.blend_duration = blend_duration
        self.check_contact = check_contact
        self.robot_name = robot_name
        self.goal_channel = goal_channel
        self.future = None
        self.sent_goal = False
        self.goal_uuid_des = None
        self.chain_index = 0
        self.completed_child_index = -1
        self.blend_reject_reason = blend_reject_reason

        # Read complex action client status from the same blackboard keys used by motion leaves.
        self.blackboard = self.attach_blackboard_client(
            name=self.name, namespace=self.robot_name,
        )
        self.goal_id_key = f"{self.goal_channel}/goal_id"
        self.goal_status_key = f"{self.goal_channel}/goal_status"
        self.blend_status_key = f"{self.goal_channel}/blend_status"
        self.blend_progress_key = f"{self.goal_channel}/blend_progress"
        self.blackboard.register_key(
            key=self.goal_id_key, access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key=self.goal_status_key, access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key=self.blend_status_key, access=py_trees.common.Access.WRITE,
        )
        self.blackboard.register_key(
            key=self.blend_progress_key, access=py_trees.common.Access.READ,
        )

    def _validate_blend(self, children, blend_duration, goal_channel, robot_name):
        """
        Validate child motions and blend timing for this composite.
        """
        # Convert blend duration once and keep any rejection reason as metadata.
        blend_reject_reason = None
        try:
            blend_duration = float(blend_duration)
        except (TypeError, ValueError) as exc:
            blend_duration = 0.0
            blend_reject_reason = f"blend_duration must be numeric: {exc}"
        if blend_reject_reason is None and blend_duration <= 0.0:
            blend_reject_reason = "blend_duration must be positive"
        if blend_reject_reason is None and goal_channel != self.arm_goal_channel:
            blend_reject_reason = "MoveBlend only supports arm goal_channel"
        if blend_reject_reason is None and len(children) < self.blend_initial_child_count:
            blend_reject_reason = "MoveBlend requires at least two children"

        # Accept only arm motion leaves that can export complex action client commands.
        if blend_reject_reason is None:
            for child in children:
                child_goal_channel = getattr(child, "goal_channel", self.arm_goal_channel)
                if child_goal_channel != self.arm_goal_channel:
                    blend_reject_reason = "MoveBlend only supports arm children"
                    break
                if not callable(getattr(child, "make_command", None)):
                    blend_reject_reason = (
                        "MoveBlend child must export a complex action client command, "
                        f"got [{type(child).__name__}]"
                    )
                    break
                child_robot_name = getattr(child, "robot_name", None)
                if child_robot_name is not None:
                    if robot_name is None:
                        robot_name = child_robot_name
                    elif child_robot_name != robot_name:
                        blend_reject_reason = "MoveBlend children must target the same robot"
                        break
                try:
                    child_timeout = float(child.timeout)
                except (TypeError, ValueError) as exc:
                    blend_reject_reason = f"MoveBlend child timeout must be numeric: {exc}"
                    break
                if child_timeout <= 0.0:
                    blend_reject_reason = "MoveBlend child timeout must be positive"
                    break
                if blend_duration >= child_timeout:
                    blend_reject_reason = "blend_duration must be shorter than each child timeout"
                    break
        return blend_duration, blend_reject_reason

    def initialise(self):
        # Reset blend execution state whenever py_trees starts this composite.
        self.future = None
        self.sent_goal = False
        self.goal_uuid_des = None
        self.chain_index = 0
        self.completed_child_index = -1
        self.blackboard.unset(self.blend_status_key)
        self.current_child = self.children[0] if self.children else None
        for child in self.children:
            child.blend_phase = None
            child.status = py_trees.common.Status.INVALID
            child.feedback_message = ""

    def current_goal_id(self):
        # Read the current goal id from the blackboard.
        try:
            return self.blackboard.get(self.goal_id_key)
        except KeyError:
            return None

    def current_goal_status(self):
        # Read the current goal status from the blackboard.
        try:
            return self.blackboard.get(self.goal_status_key)
        except KeyError:
            return None

    def current_blend_progress(self):
        # Read the current blend progress JSON from the blackboard.
        try:
            progress = self.blackboard.get(self.blend_progress_key)
        except KeyError:
            return None
        if not progress:
            return None
        try:
            return json.loads(progress)
        except (TypeError, ValueError):
            return None

    def goal_id_matches_blackboard(self):
        # Match the blackboard goal id against the currently sent blend step.
        goal_id = self.current_goal_id()
        if goal_id is None or self.goal_uuid_des is None:
            return False
        match = self.goal_uuid_des == goal_id
        return match.all() if hasattr(match, "all") else bool(match)

    def command_response_status(self):
        # Convert immediate complex action client failures into a BT failure.
        if self.future is None or not self.future.done():
            return None

        try:
            response = self.future.result()
        except Exception as exc:
            self.feedback_message = f"command service failed: {exc}"
            return py_trees.common.Status.FAILURE

        goal_status = getattr(response, "goal_status", None)
        if goal_status is None:
            return None
        status = getattr(goal_status, "status", GoalStatus.STATUS_UNKNOWN)
        failure_statuses = [
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_UNKNOWN,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_CANCELED,
        ]
        if status not in failure_statuses:
            return None

        response_uuid = list(getattr(goal_status.goal_info.goal_id, "uuid", []))
        if self.goal_uuid_des is not None and response_uuid:
            if response_uuid != list(self.goal_uuid_des):
                return None
        elif self.goal_uuid_des is not None:
            return None

        self.feedback_message = "FAILURE"
        return py_trees.common.Status.FAILURE

    def command_from_child(self, child):
        # Assume _validate_blend already verified make_command exists.
        command = child.make_command(enable_wait=False)

        # Blend children share the parent blend goal id and never wait independently.
        command.pop("uuid", None)
        command["enable_wait"] = False
        return command

    def _send_current_blend(self, commands):
        # Send the first two commands once, then only the next command per handoff.
        self.goal_uuid_des = np.random.randint(0, 255, size=16, dtype=np.uint8)
        if self.chain_index == 0:
            blend_children = commands[:self.blend_initial_child_count]
        else:
            blend_children = commands[
                self.chain_index + 1:self.chain_index + 1 + self.blend_next_child_count
            ]
        blend_goal = {
            "blend": True,
            "chain_index": self.chain_index,
            "chain_total": len(commands) - 1,
            "children": blend_children,
        }
        cmd_str = json.dumps(
            {
                'action_type': self.blend_action_type,
                'goal': json.dumps(blend_goal),
                'uuid': self.goal_uuid_des.tolist(),
                'goal_channel': self.goal_channel,
                'timeout': self.timeout,
                'blend_duration': self.blend_duration,
                'check_contact': self.check_contact,
                'enable_wait': False
            }
        )
        req = StringGoalStatus.Request(data=cmd_str)
        self.future = self.cmd_req.call_async(req)

        # Mark this blend step as in-flight for status sync and tip tracking.
        self.sent_goal = True
        self.feedback_message = f"Sending blend step {self.chain_index + 1}"

    def sync_child_statuses(self, new_status):
        # Reflect the current blend state onto child action markers.
        self.blackboard.unset(self.blend_status_key)

        # Mark active blend children with blend metadata and py_trees status.
        running_indexes = {self.chain_index, self.chain_index + 1}
        for index, child in enumerate(self.children):
            if new_status == py_trees.common.Status.SUCCESS:
                # Mark every child done when the whole blend succeeds.
                child.blend_phase = None
                child.status = py_trees.common.Status.SUCCESS
                child.feedback_message = ""
            elif new_status == py_trees.common.Status.FAILURE and index in running_indexes:
                # Mark only the active blend window failed on blend failure.
                child.blend_phase = None
                child.status = py_trees.common.Status.FAILURE
                child.feedback_message = ""
            elif index <= self.completed_child_index:
                # Keep already handed-off children successful.
                child.blend_phase = None
                child.status = py_trees.common.Status.SUCCESS
                child.feedback_message = ""
            elif (
                new_status == py_trees.common.Status.RUNNING
                and self.sent_goal
                and index == self.chain_index
            ):
                # Show the first child of the active blend window as blending out.
                child.blend_phase = BlendPhase.RUNNING_BLEND_FIRST
                child.status = py_trees.common.Status.RUNNING
                child.feedback_message = child.blend_phase.name
                self.blackboard.set(self.blend_status_key, child.blend_phase.value)
            elif (
                new_status == py_trees.common.Status.RUNNING
                and self.sent_goal
                and index == self.chain_index + 1
            ):
                # Show the second child of the active blend window as blending in.
                child.blend_phase = BlendPhase.RUNNING_BLEND_SECOND
                child.status = py_trees.common.Status.RUNNING
                child.feedback_message = child.blend_phase.name
            else:
                # Clear children that are not active yet.
                child.blend_phase = None
                child.status = py_trees.common.Status.INVALID
                child.feedback_message = ""

        # Point py_trees visitors at a non-invalid child so root.tip() stays valid.
        # This is for tree status display/debugging, not command dispatch or blend logic.
        if not self.children:
            self.current_child = None
        elif new_status == py_trees.common.Status.RUNNING and self.sent_goal:
            if self.completed_child_index == self.chain_index:
                self.current_child = self.children[self.chain_index + 1]
            else:
                self.current_child = self.children[self.chain_index]
        elif new_status == py_trees.common.Status.SUCCESS:
            self.current_child = self.children[-1]
        else:
            self.current_child = self.children[self.chain_index]

    def tick(self) -> typing.Iterator[py_trees.behaviour.Behaviour]:
        # Drive the blend composite without ticking child actions directly.
        self.logger.debug(f"{self.__class__.__name__}.tick()")
        if self.status != py_trees.common.Status.RUNNING:
            for child in self.children:
                if child.status != py_trees.common.Status.INVALID:
                    child.stop(py_trees.common.Status.INVALID)
            self.initialise()

        # Fail this composite if any child is already marked failed.
        for child in self.children:
            if child.status == py_trees.common.Status.FAILURE:
                self.feedback_message = f"Blend child [{child.name}] failed"
                new_status = py_trees.common.Status.FAILURE
                self.sync_child_statuses(new_status)
                self.stop(new_status)
                yield self
                return

        # Reject unavailable complex action client transport.
        if self.cmd_req is None:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            new_status = py_trees.common.Status.FAILURE
            self.sync_child_statuses(new_status)
            self.stop(new_status)
            yield self
            return

        # Get commands from children already checked by blend validation.
        try:
            commands = [self.command_from_child(child) for child in self.children]
        except Exception as exc:
            self.feedback_message = f"Blend command build failed: {exc}"
            new_status = py_trees.common.Status.FAILURE
            self.sync_child_statuses(new_status)
            self.stop(new_status)
            yield self
            return

        # Send the first blend step.
        if not self.sent_goal:
            self._send_current_blend(commands)
            new_status = py_trees.common.Status.RUNNING
            self.sync_child_statuses(new_status)
            self.status = new_status
            for child in self.children[self.chain_index:self.chain_index + self.blend_initial_child_count]:
                yield child
            yield self
            return

        # Handle immediate complex action client rejection before waiting for status topics.
        command_status = self.command_response_status()
        if command_status is not None:
            self.sync_child_statuses(command_status)
            self.stop(command_status)
            yield self
            return

        # Keep running until complex action client publishes the goal id for this blend step.
        if self.current_goal_id() is None:
            new_status = py_trees.common.Status.RUNNING
            self.sync_child_statuses(new_status)
            self.status = new_status
            for child in self.children[self.chain_index:self.chain_index + self.blend_initial_child_count]:
                yield child
            yield self
            return

        # Fail only when the blackboard status belongs to this blend goal id.
        if (
            self.goal_id_matches_blackboard() and \
            self.current_goal_status() in (
                GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_UNKNOWN, 
                GoalStatus.STATUS_CANCELING, GoalStatus.STATUS_CANCELED
            )
        ):
            self.feedback_message = "FAILURE"
            new_status = py_trees.common.Status.FAILURE
            self.sync_child_statuses(new_status)
            self.stop(new_status)
            yield self
            return

        # Consume blend progress only when it reports this blend step uuid.
        progress = self.current_blend_progress()
        if (
            progress is not None
            and self.goal_uuid_des is not None
            and progress.get("uuid") == list(self.goal_uuid_des)
        ):
            progress_status = progress.get("status")
            if progress_status in (
                BlendProgressStatus.ABORTED.value,
                BlendProgressStatus.CANCELED.value,
            ):
                self.feedback_message = progress.get("message", "FAILURE")
                new_status = py_trees.common.Status.FAILURE
                self.sync_child_statuses(new_status)
                self.stop(new_status)
                yield self
                return

            # Advance one child when complex action client reports target tolerance.
            completed_child_index = int(progress.get("completed_child_index", -1))
            if (
                progress_status == BlendProgressStatus.HANDOFF.value
                and completed_child_index > self.completed_child_index
            ):
                self.completed_child_index = completed_child_index
                if completed_child_index < len(commands) - 2:
                    self.chain_index = completed_child_index + 1
                    self.sent_goal = False
                    self.future = None
                    self._send_current_blend(commands)
                self.feedback_message = f"Blend child {completed_child_index + 1} succeeded"

        # Finish after the final complex action client blend step succeeds.
        if (
            self.goal_id_matches_blackboard() 
            and self.current_goal_status() == GoalStatus.STATUS_SUCCEEDED
        ):
            self.completed_child_index = len(commands) - 1
            self.feedback_message = "SUCCESSFUL"
            new_status = py_trees.common.Status.SUCCESS
            self.sync_child_statuses(new_status)
            self.stop(new_status)
            yield self
            return

        # Keep this composite running while the current blend step is active.
        new_status = py_trees.common.Status.RUNNING
        self.sync_child_statuses(new_status)
        self.status = new_status
        for child in self.children[self.chain_index:self.chain_index + self.blend_initial_child_count]:
            yield child
        yield self

    def terminate(self, new_status):
        # Cancel the active arm goal when this composite is interrupted.
        self.logger.debug("%s.terminate()" % self.__class__.__name__)
        if new_status == py_trees.common.Status.SUCCESS:
            return
        if self.current_goal_id() is None:
            self.feedback_message = "goal_id is not available"
            return

        status = self.current_goal_status()
        if (
            self.goal_id_matches_blackboard()  
            and status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        ):
            req = StringGoalStatus.Request()
            req.data = json.dumps({'action_type': 'cancel_goal',
                                   'goal_channel': self.goal_channel,
                                   'enable_wait': True})
            self.future = self.cmd_req.call_async(req)
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))
