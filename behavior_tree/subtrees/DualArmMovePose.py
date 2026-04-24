import json

import numpy as np
import py_trees
from action_msgs.msg import GoalStatus
from riro_srvs.srv import StringGoalStatus

from . import Move


class DualArmMoveP(Move.MOVE):
    def __init__(
        self,
        name,
        action_client,
        action_type,
        action_goal=None,
        timeout=3.0,
        extra=None,
    ):
        super(DualArmMoveP, self).__init__(
            name=name,
            action_client=action_client,
            action_goal=action_goal,
            timeout=timeout,
        )
        self.action_type = action_type
        self.extra = dict(extra or {})
        if isinstance(action_goal, str):
            self.blackboard.register_key(
                key=action_goal,
                access=py_trees.common.Access.READ,
            )

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = "no action client, did you call setup() on your tree?"
            return py_trees.common.Status.FAILURE

        if not self.sent_goal:
            self.goal_uuid_des = np.random.randint(0, 255, size=16, dtype=np.uint8)
            command = dict(self.extra)
            command.update(
                {
                    "action_type": self.action_type,
                    "uuid": self.goal_uuid_des.tolist(),
                    "timeout": self.timeout,
                    "enable_wait": False,
                }
            )

            goal = self._resolve_goal()
            if goal is not None:
                command["goal"] = goal if isinstance(goal, str) else json.dumps(goal)

            self.future = self.cmd_req.call_async(
                StringGoalStatus.Request(data=json.dumps(command))
            )
            self.sent_goal = True
            self.feedback_message = f"Sending {self.action_type} goal"
            return py_trees.common.Status.RUNNING

        if self.blackboard.goal_id is None:
            return py_trees.common.Status.RUNNING

        if not (self.goal_uuid_des == self.blackboard.goal_id).all():
            return py_trees.common.Status.RUNNING

        if self.blackboard.goal_status == GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            return py_trees.common.Status.SUCCESS

        if self.blackboard.goal_status in [
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_UNKNOWN,
            GoalStatus.STATUS_CANCELING,
            GoalStatus.STATUS_CANCELED,
        ]:
            self.feedback_message = "FAILURE"
            return py_trees.common.Status.FAILURE

        self.feedback_message = "running"
        return py_trees.common.Status.RUNNING

    def _resolve_goal(self):
        if isinstance(self.action_goal, str):
            return self.blackboard.get(self.action_goal)
        return self.action_goal
