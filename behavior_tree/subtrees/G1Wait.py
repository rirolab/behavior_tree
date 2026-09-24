"""Behavior-tree wait leaf used between G1 command steps."""

from __future__ import annotations

import time

import py_trees


class WaitDuration(py_trees.behaviour.Behaviour):
    """Return success after the configured duration has elapsed."""

    def __init__(self, name, duration):
        super().__init__(name=name)
        self.duration = float(duration)
        self._deadline = None

    def initialise(self):
        self._deadline = time.monotonic() + self.duration
        self.feedback_message = f"waiting {self.duration:.3f} seconds"

    def update(self):
        if self._deadline is None:
            return py_trees.common.Status.FAILURE
        remaining = self._deadline - time.monotonic()
        if remaining <= 0.0:
            self.feedback_message = "wait complete"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = f"{remaining:.3f} seconds remaining"
        return py_trees.common.Status.RUNNING
