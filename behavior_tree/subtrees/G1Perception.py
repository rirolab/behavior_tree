"""Trigger one head-camera IKEA world-model snapshot from the behavior tree."""

from __future__ import annotations

import time

import py_trees
from std_srvs.srv import Trigger


class UpdateWorldModel(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_name, timeout):
        super().__init__(name=name)
        self.service_name = str(service_name)
        self.timeout = float(timeout)
        self._client = None
        self._future = None
        self._deadline = None
        self._retry_after = -float("inf")

    def setup(self, node):
        self._client = node.create_client(Trigger, self.service_name)

    def initialise(self):
        self._future = None
        self._deadline = time.monotonic() + self.timeout
        self._retry_after = -float("inf")

    def update(self):
        if self._client is None:
            self.feedback_message = "world-model snapshot client was not set up"
            return py_trees.common.Status.FAILURE
        if self._future is None:
            if time.monotonic() < self._retry_after:
                return py_trees.common.Status.RUNNING
            if self._client.service_is_ready():
                self._future = self._client.call_async(Trigger.Request())
                self.feedback_message = "capturing IKEA head-camera scene"
            elif time.monotonic() >= self._deadline:
                self.feedback_message = f"{self.service_name} unavailable"
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        if self._future.done():
            try:
                response = self._future.result()
            except Exception as exc:
                self.feedback_message = f"world-model snapshot failed: {exc}"
                return py_trees.common.Status.FAILURE
            self.feedback_message = response.message
            if response.success:
                return py_trees.common.Status.SUCCESS
            if time.monotonic() >= self._deadline:
                return py_trees.common.Status.FAILURE
            self._future = None
            self._retry_after = time.monotonic() + 0.3
            return py_trees.common.Status.RUNNING
        if time.monotonic() >= self._deadline:
            self.feedback_message = "world-model snapshot timed out"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.INVALID and self._future is not None:
            self._future.cancel()
