import time
import threading

import py_trees
from std_srvs.srv import Trigger


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


class WAIT_UNTIL_TRIGGER(py_trees.behaviour.Behaviour):
    """
    Wait until a Trigger service request is received.

    The behaviour opens a ``std_srvs/srv/Trigger`` server during setup and
    returns RUNNING until that service is called. Each new execution resets the
    trigger flag, so the trigger should be sent while this behaviour is active.
    """

    def __init__(
        self,
        name,
        trigger_service_name="~/wait_until_trigger/continue",
        robot_name=None,
    ):
        """
        Initialise a trigger wait behaviour.

        Args:
            name (:obj:`str`): behaviour name.
            trigger_service_name (:obj:`str`): Trigger service name to create.
            robot_name (:obj:`str`): optional blackboard namespace.
        """
        super(WAIT_UNTIL_TRIGGER, self).__init__(name=name)
        self.trigger_service_name = trigger_service_name
        self.trigger_service = None
        self.triggered = False
        self._lock = threading.Lock()
        self.blackboard = self.attach_blackboard_client(
            name=self.name,
            namespace=robot_name,
        )

    def setup(self, node):
        """
        Create the Trigger service used to release this behaviour.
        """
        self.trigger_service = node.create_service(
            Trigger,
            self.trigger_service_name,
            self.trigger_callback,
        )
        self.feedback_message = f"waiting for trigger service [{self.trigger_service_name}]"

    def initialise(self):
        """
        Reset the trigger flag for this execution.
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)
        with self._lock:
            self.triggered = False
        self.feedback_message = f"waiting for trigger service [{self.trigger_service_name}]"

    def update(self):
        """
        Return running until the Trigger service has been called.

        Returns:
            :class:`~py_trees.common.Status`: behaviour status.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        if self.trigger_service is None:
            self.feedback_message = "trigger service was not initialised"
            return py_trees.common.Status.FAILURE

        with self._lock:
            triggered = self.triggered

        if triggered:
            self.feedback_message = f"trigger received from [{self.trigger_service_name}]"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = f"waiting for trigger service [{self.trigger_service_name}]"
        return py_trees.common.Status.RUNNING

    def trigger_callback(self, request, response):
        """
        Mark this behaviour as triggered.
        """
        del request
        with self._lock:
            self.triggered = True
        response.success = True
        response.message = f"{self.name} triggered"
        return response
