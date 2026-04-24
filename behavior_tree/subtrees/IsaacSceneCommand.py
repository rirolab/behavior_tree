import json
import random
import time

import py_trees
from std_msgs.msg import String


class ISAAC_SCENE_COMMAND(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name,
        command_topic="/scene/command",
        status_topic="/scene/command_status",
        command=None,
        timeout=2.0,
    ):
        super(ISAAC_SCENE_COMMAND, self).__init__(name=name)
        self.command_topic = command_topic
        self.status_topic = status_topic
        self.command = command
        self.timeout = float(timeout)
        self.node = None
        self.publisher = None
        self.subscription = None
        self.sent_goal = False
        self.deadline = None
        self.goal_uuid = None
        self.result_status = None
        self.blackboard = self.attach_blackboard_client(name=self.name)
        if isinstance(command, str):
            self.blackboard.register_key(
                key=command,
                access=py_trees.common.Access.READ,
            )

    def setup(self, node):
        self.node = node
        self.publisher = node.create_publisher(String, self.command_topic, 10)
        self.subscription = node.create_subscription(
            String,
            self.status_topic,
            self._status_callback,
            10,
        )

    def initialise(self):
        self.sent_goal = False
        self.deadline = time.monotonic() + self.timeout
        self.goal_uuid = [random.randrange(0, 256) for _ in range(16)]
        self.result_status = None

    def update(self):
        if self.publisher is None:
            self.feedback_message = "scene command publisher is not initialized"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            if self.publisher.get_subscription_count() == 0:
                if time.monotonic() < self.deadline:
                    self.feedback_message = f"waiting for {self.command_topic}"
                    return py_trees.common.Status.RUNNING
                self.feedback_message = f"{self.command_topic} has no subscribers"
                return py_trees.common.Status.FAILURE

            command = dict(self._resolve_command())
            command["uuid"] = self.goal_uuid
            msg = String()
            msg.data = json.dumps(command)
            self.publisher.publish(msg)
            self.sent_goal = True
            self.feedback_message = f"sent scene command {command.get('action_type')}"
            return py_trees.common.Status.RUNNING

        if self.result_status == "succeeded":
            self.feedback_message = "scene command succeeded"
            return py_trees.common.Status.SUCCESS
        if self.result_status == "aborted":
            self.feedback_message = "scene command failed"
            return py_trees.common.Status.FAILURE

        if time.monotonic() < self.deadline:
            return py_trees.common.Status.RUNNING
        self.feedback_message = "scene command timed out"
        return py_trees.common.Status.FAILURE

    def _resolve_command(self):
        if isinstance(self.command, str):
            return self.blackboard.get(self.command)
        return self.command or {}

    def _status_callback(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if status.get("uuid") == self.goal_uuid:
            self.result_status = status.get("status")
