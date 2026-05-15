import time

import py_trees


class WAIT(py_trees.behaviour.Behaviour):
    """
    Wait for a fixed amount of time before succeeding.

    The duration can be provided directly as a number of seconds or indirectly
    via a blackboard key.
    """

    def __init__(self, name, duration=1.0, robot_name=None):
        """
        Initialise a wait behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            duration (:obj:`float` or :obj:`str`): wait duration in seconds, or
                blackboard key containing the duration.
            robot_name (:obj:`str`): optional blackboard namespace.
        """
        super(WAIT, self).__init__(name=name)
        self.duration = duration
        self.resolved_duration = None
        self.deadline = None
        self.duration_error = None
        self.blackboard = self.attach_blackboard_client(
            name=self.name,
            namespace=robot_name,
        )
        if isinstance(duration, str):
            self.blackboard.register_key(
                key=duration,
                access=py_trees.common.Access.READ,
            )

    def initialise(self):
        """
        Resolve the duration and start the timer for this execution.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        self.duration_error = None
        self.resolved_duration = None
        self.deadline = None
        try:
            self.resolved_duration = self.resolve_duration()
        except (KeyError, TypeError, ValueError) as error:
            self.duration_error = str(error)
            self.feedback_message = self.duration_error
            return

        self.deadline = time.monotonic() + self.resolved_duration
        self.feedback_message = f"waiting for {self.resolved_duration:.3f}s"

    def update(self):
        """
        Return running until the configured duration has elapsed.

        Returns:
            :class:`~py_trees.common.Status`: behaviour status.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        if self.duration_error is not None:
            return py_trees.common.Status.FAILURE

        if self.deadline is None:
            self.feedback_message = "wait timer was not initialised"
            return py_trees.common.Status.FAILURE

        remaining = self.deadline - time.monotonic()
        if remaining <= 0.0:
            self.feedback_message = f"waited {self.resolved_duration:.3f}s"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"waiting ({remaining:.3f}s left)"
        return py_trees.common.Status.RUNNING

    def resolve_duration(self):
        """
        Resolve and validate the duration value.

        Returns:
            :obj:`float`: wait duration in seconds.
        """
        duration = self.blackboard.get(self.duration) if isinstance(self.duration, str) else self.duration

        if not isinstance(duration, (int, float)):
            raise TypeError(
                f"{self.name}: duration should be int or float, got {type(duration).__name__}"
            )
        if duration < 0.0:
            raise ValueError(f"{self.name}: duration should be non-negative")

        return float(duration)
