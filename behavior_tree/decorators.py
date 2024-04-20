import time
from py_trees.decorators import *
import py_trees

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
    

class FTReplanning(Decorator):
    """
    A decorator that loops the node or branch a number of times, or infinitely.
    """
    def __init__(self, child, idx, name=common.Name.AUTO_GENERATED):
        """
        Init with the decorated child.
        
        Args:
        child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
        name (:obj:`str`): the decorator name
        """
        super(FTReplanning, self).__init__(name=name, child=child)
        self.idx = idx

    # def setup(self, timeout):
    #     return super(FTReplanning, self).setup(timeout)

    def initialise(self):
        """
        Reset the feedback message and finish time on behaviour entry.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.logger.info("FTReplanning Initialise!\n\n\n\n")
        self.feedback_message = "[(Decorator) FTReplanning]"
        self.sent_goal = False
        self.blackboard = py_trees.blackboard.Client()
        self.is_recover = False
        self.blackboard.register_key(key= 'recover_flag', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key= 'recover_flag', access=py_trees.common.Access.READ)
        self.blackboard.set('recover_flag', self.is_recover)

        self.blackboard.register_key(key= 'reinsert_condition', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key= 'reinsert_condition', access=py_trees.common.Access.READ)
        self.blackboard.set('reinsert_condition', False)

        self.blackboard.register_key(key= 'goal_reached', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key= 'goal_reached', access=py_trees.common.Access.READ)
        self.blackboard.set('goal_reached', False)
        self.is_goal_reached = False


    def update(self):
        """
        Flip :data:`~py_trees.common.Status.FAILURE` and
        :data:`~py_trees.common.Status.SUCCESS`
        
        Returns:
        :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.is_recover = self.blackboard.get('recover_flag')
        self.is_goal_reached = self.blackboard.get('goal_reached')

        if self.is_goal_reached:
            if self.decorated.status == common.Status.SUCCESS:
                return common.Status.SUCCESS

        if not self.sent_goal:
            self.decorated.initialise()
            self.sent_goal = True
            return common.Status.RUNNING

        if self.decorated.status == common.Status.FAILURE:
            return common.Status.FAILURE

        if self.is_recover:
            self.decorated.initialise()
            self.blackboard.set('recover_flag', False)
            self.blackboard.set('reinsert_condition', True)
            return common.Status.RUNNING


        return common.Status.RUNNING

    
class ConditionalLoop(Decorator):
    """
    A decorator that loops the node or branch a number of times, or infinitely.
    """
    def __init__(self, child, idx, name=common.Name.AUTO_GENERATED):
        """
        Init with the decorated child.
        
        Args:
        child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
        name (:obj:`str`): the decorator name
        n_loop: the number of loops to run
        enable_inf_loop: run indefinitely
        timeout: timeout value if enalble_inf_loop is True (a negative timeout will loop forever).
        """
        super(ConditionalLoop, self).__init__(name=name, child=child)

        self.idx = idx

    def initialise(self):
        """
        Reset the feedback message and finish time on behaviour entry.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))

        self.feedback_message = "[(Decorator) FTReplanning]"
        self.sent_goal = False
        self.blackboard = py_trees.blackboard.Client()

        self.blackboard.register_key(key= 'reinsert_condition', access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key= 'reinsert_condition', access=py_trees.common.Access.READ)
        self.blackboard.set('reinsert_condition', False)
        self.is_reinsert = False
        
    def update(self):
        """
        Flip :data:`~py_trees.common.Status.FAILURE` and
        :data:`~py_trees.common.Status.SUCCESS`
        
        Returns:
        :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.is_reinsert = self.blackboard.get('reinsert_condition')

        if not self.sent_goal:
            self.decorated.initialise()
            self.sent_goal = True
            return common.Status.RUNNING

        if self.decorated.status == common.Status.FAILURE:
            return common.Status.FAILURE

        if self.is_reinsert:
            self.decorated.initialise()
            self.blackboard.set('reinsert_condition', False)
            return common.Status.RUNNING

        return common.Status.RUNNING


class ConditionalRUN(Decorator):
    """
    A decorator that loops the node or branch a number of times, or infinitely.
    """
    def __init__(self, child, idx, name=common.Name.AUTO_GENERATED):
        """
        Init with the decorated child.
        
        Args:
        child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
        name (:obj:`str`): the decorator name
        n_loop: the number of loops to run
        enable_inf_loop: run indefinitely
        timeout: timeout value if enalble_inf_loop is True (a negative timeout will loop forever).
        """
        super(ConditionalRUN, self).__init__(name=name, child=child)
        self.idx = idx
        self.bb_name = 'conditional_run_' + idx

    def initialise(self):
        """
        Reset the feedback message and finish time on behaviour entry.
        """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))

        self.feedback_message = "[(Decorator) ConditionalRUN]"
        self.sent_goal = False
        self.blackboard = py_trees.blackboard.Client()

        self.blackboard.register_key(key= self.bb_name, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key= self.bb_name, access=py_trees.common.Access.READ)
        
    def update(self):
        """
        Flip :data:`~py_trees.common.Status.FAILURE` and
        :data:`~py_trees.common.Status.SUCCESS`
        
        Returns:
        :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if not self.blackboard.get(self.bb_name):
            return common.Status.SUCCESS
        else:
            if not self.sent_goal:
                self.decorated.initialise()
                self.sent_goal = True
                return common.Status.RUNNING

            if self.decorated.status == common.Status.SUCCESS:
                return common.Status.SUCCESS

            if self.decorated.status == common.Status.FAILURE:
                return common.Status.FAILURE

            if self.is_reinsert:
                self.decorated.initialise()
                self.blackboard.set('reinsert_condition', False)
                return common.Status.RUNNING

            return common.Status.RUNNING
