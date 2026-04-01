import json

import py_trees
import py_trees.console as console
import std_msgs.msg as std_msgs

from . import base_job
from behavior_tree.subtrees import Policy


class Move(base_job.BaseJob):
    """
    Job handler for policy execution steps.
    """

    def __init__(self, node):
        super(Move, self).__init__(node)

    def incoming(self, msg):
        """
        Cache the grounding when any step requests policy execution.
        """
        if self.goal:
            self._node.get_logger().error("policy_job: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)["params"]
            for i in range(len(grounding.keys())):
                if grounding[str(i + 1)]["primitive_action"] in ["policy_execute"]:
                    self.goal = grounding
                    break

    @staticmethod
    def create_root(action_client, idx="1", goal=std_msgs.Empty(), **kwargs):
        """
        Create a subtree for a single policy execution step.
        """
        if goal[idx]["primitive_action"] not in ["policy_execute"]:
            return None

        if not goal[idx].get("policy_name"):
            console.logerror("Policy: No policy_name provided")
            return None

        root = py_trees.composites.Sequence(name="Policy", memory=True)
        run_policy = Policy.RUN(
            name="RunPolicy",
            action_client=action_client,
            action_goal=goal[idx],
            timeout=float(goal[idx].get("timeout_sec", 5.0)),
        )
        root.add_child(run_policy)
        return root
