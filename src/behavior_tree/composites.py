import time
import py_trees
from py_trees.composites import *
import rospy


##############################################################################
# Composites
##############################################################################


class PriorityParallel(Composite):
    """
    return Status.Success until the first child is succeeded.
    """
    def __init__(self, name="PriorityParallel", policy="SUCCESS_ON_PRIORITIZED_CHILD", children=None, *args, **kwargs):
        super(PriorityParallel, self).__init__(name, children, *args, **kwargs)
        self.policy = policy
        self.prioritized_child = self.children[0]

    def tick(self):
        """
        Tick over the children.

        Yields:
            :class:`~py_trees.behaviour.Behaviour`: a reference to itself or one of its children
        """
        if self.status != Status.RUNNING:
            # subclass (user) handling
            self.initialise()
        self.logger.debug("%s.tick()" % self.__class__.__name__)
        # process them all first
        for child in self.children:
            for node in child.tick():
                yield node
        # new_status = Status.SUCCESS if self.policy == common.ParallelPolicy.SUCCESS_ON_ALL else Status.RUNNING
        new_status = Status.RUNNING
        if any([c.status == Status.FAILURE for c in self.children]):
            new_status = Status.FAILURE
        else:
            if self.policy == "SUCCESS_ON_PRIORITIZED_CHILD":
                if self.prioritized_child.status == Status.SUCCESS:
                    new_status = Status.SUCCESS
            else:
                raise NotImplementedError('This is only for prioritized parallel composite.')
                
        # special case composite - this parallel may have children that are still running
        # so if the parallel itself has reached a final status, then these running children
        # need to be made aware of it too
        if new_status != Status.RUNNING:
            for child in self.children:
                if child.status == Status.RUNNING:
                    # interrupt it (exactly as if it was interrupted by a higher priority)
                    child.stop(Status.INVALID)
            self.stop(new_status)
        self.status = new_status
        yield self

    @property
    def current_child(self):
        """
        Have to check if there's anything actually running first.

        Returns:
            :class:`~py_trees.behaviour.Behaviour`: the child that is currently running, or None
        """
        if self.status == Status.INVALID:
            return None
        if self.status == Status.FAILURE:
            for child in self.children:
                if child.status == Status.FAILURE:
                    return child
            # shouldn't get here
        elif self.status == Status.SUCCESS and self.policy == "SUCCESS_ON_PRIORITIZED_CHILD+":
            for child in self.children:
                if child.status == Status.SUCCESS:
                    return child
        else:
            return self.children[-1]

    def add_child(self, child):
        """
        Adds a child.

        Args:
            child (:class:`~py_trees.behaviour.Behaviour`): child to add

        Returns:
            uuid.UUID: unique id of the child
        """
        assert isinstance(child, Behaviour), "children must be behaviours, but you passed in %s" % type(child)
        self.children.append(child)
        child.parent = self
        self.prioritized_child = self.children[0]
        return child.id

    def add_children(self, children):
        """
        Append a list of children to the current list.

        Args:
            children ([:class:`~py_trees.behaviour.Behaviour`]): list of children to add
        """
        for child in children:
            self.add_child(child)
