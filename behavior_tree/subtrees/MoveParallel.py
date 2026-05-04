import py_trees


class MoveParallel(py_trees.composites.Parallel):
    """
    Simple parallel subtree wrapper for multi-robot motions.
    """

    def __init__(self, name, children):
        """
        Create a parallel composite that succeeds when all children succeed.

        Args:
            name (:obj:`str`): subtree name.
            children ([:class:`py_trees.behaviour.Behaviour`]): child behaviours.
        """
        super(MoveParallel, self).__init__(
            name=name,
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=True,
            ),
        )
        self.add_children(children)
