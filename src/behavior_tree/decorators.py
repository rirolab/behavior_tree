import time
from py_trees.decorators import *

class Loop(Decorator):
    """
    A decorator that loops the node or branch a number of times, or infinitely.
    """
    def __init__(self, child, name=common.Name.AUTO_GENERATED, n_loop=100, enable_inf_loop=False,
                 timeout=-1, condition=None, return_success=False):
        """
        Init with the decorated child.
        
        Args:
        child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
        name (:obj:`str`): the decorator name
        n_loop: the number of loops to run
        enable_inf_loop: run indefinitely
        timeout: timeout value if enalble_inf_loop is True (a negative timeout will loop forever).
        """
        super(Loop, self).__init__(name=name, child=child)

        self.n_loop          = n_loop
        self.enable_inf_loop = enable_inf_loop
        self.duration        = timeout
        self.condition       = condition
        self.finish_time     = None
        self.return_success  = return_success

    def initialise(self):
        """
        Reset the feedback message and finish time on behaviour entry.
        """
        self.finish_time = time.time() + self.duration
        self.feedback_message = ""
        self.count = 0
        
    def update(self):
        """
        Flip :data:`~py_trees.common.Status.FAILURE` and
        :data:`~py_trees.common.Status.SUCCESS`
        
        Returns:
        :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """

        current_time = time.time()
        if self.enable_inf_loop and self.duration >= 0 and current_time > self.finish_time:
            self.feedback_message = "timed out"
            self.logger.debug("{}.update() {}".format(self.__class__.__name__, self.feedback_message))
            # invalidate the decorated (i.e. cancel it), could also put this logic in a terminate() method
            self.decorated.stop(common.Status.INVALID)
            return common.Status.FAILURE

        if self.decorated.status == common.Status.SUCCESS:
            self.count += 1
            if self.condition is not None and self.decorated.status == self.condition:
                return common.Status.SUCCESS                
            if self.enable_inf_loop is False and self.count > self.n_loop:
                return common.Status.SUCCESS
            
            if self.enable_inf_loop:
                self.feedback_message = self.decorated.feedback_message + " [Infinite loop]"
            else:
                self.feedback_message = self.decorated.feedback_message + " [n_loop: {}/{}]".format(self.count, self.n_loop)
            return common.Status.RUNNING

        if self.enable_inf_loop:
            self.feedback_message = self.decorated.feedback_message + " [Infinite loop]"
            return common.Status.RUNNING
        else:
            if self.condition is not None and self.decorated.status != self.condition:
                self.feedback_message = self.decorated.feedback_message + " [loop under condition]"
                return common.Status.RUNNING
                            
            self.feedback_message = self.decorated.feedback_message
            return self.decorated.status
    
