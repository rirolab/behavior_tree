import py_trees
from std_msgs.msg import Bool


class RUN_WITH_BOOL_TRIGGER(py_trees.decorators.Decorator):
    """
    Publish a Bool trigger while the decorated child is running.
    """

    def __init__(self, name, child, topic_name, start_value=True, stop_value=False):
        super(RUN_WITH_BOOL_TRIGGER, self).__init__(name=name, child=child)

        # Store the trigger topic and payloads used around child execution.
        self.topic_name = str(topic_name or "").strip()
        self.start_value = bool(start_value)
        self.stop_value = bool(stop_value)
        self.publisher = None
        self._started = False

    def setup(self, node):
        # Create the publisher once the BT runtime provides the ROS node.
        if not self.topic_name:
            raise RuntimeError(f"{self.name}: topic_name must not be empty")
        self.publisher = node.create_publisher(Bool, self.topic_name, 10)

    def initialise(self):
        # Publish the start state before the child gets its first tick.
        self._started = True
        self._publish(self.start_value)
        self.feedback_message = f"triggered {self.topic_name}={self.start_value}"

    def update(self):
        # Mirror the decorated child's status so success and failure are preserved.
        if self.decorated.status == py_trees.common.Status.RUNNING:
            self.feedback_message = self.decorated.feedback_message
            return py_trees.common.Status.RUNNING
        self.feedback_message = self.decorated.feedback_message
        return self.decorated.status

    def terminate(self, new_status):
        # Publish the stop state on success, failure, or interruption.
        if self._started:
            self._publish(self.stop_value)
            self._started = False
            self.feedback_message = f"triggered {self.topic_name}={self.stop_value}"

    def _publish(self, value):
        # Send the configured Bool state to the helper node.
        if self.publisher is None:
            raise RuntimeError(f"{self.name}: publisher is not ready")
        self.publisher.publish(Bool(data=bool(value)))
