import py_trees
import py_trees_ros
import rclpy
import json
import std_msgs.msg as std_msgs
from action_msgs.msg import GoalStatus

from py_trees_ros import subscribers
import py_trees.console as console

class ToBlackboard(subscribers.ToBlackboard):

    def __init__(self,
                 name: str,
                 topic_name: str):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=GoalStatus,
                                           blackboard_variables={"uuid": "goal_info.goal_id.uuid", "goal_status": "status", "goal_info": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
                                           qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                           )

        self.blackboard.register_key(key="uuid", access=py_trees.common.Access.WRITE)        
        self.blackboard.register_key(key="goal_status", access=py_trees.common.Access.READ)        
        self.blackboard.register_key(key="goal_id", access=py_trees.common.Access.WRITE)
        self.blackboard.uuid = None


    def update(self) -> py_trees.common.Status:
        """
        Call the parent to write the raw data to the blackboard.

        Returns:
            :attr:`~py_trees.common.Status.SUCCESS` if a message was written, :attr:`~py_trees.common.Status.RUNNING` otherwise.
        """        
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        
        if status != py_trees.common.Status.RUNNING:

            # we got something
            if self.blackboard.uuid is None:
                self.feedback_message = "%s: No goal_info on the blackboard!" % self.name
                return status
            s = self.blackboard.uuid

            console.loginfo(str(s[0]))
            console.loginfo(str(type(s)))
            #list_of_strings.split(" ")
            from IPython import embed; embed(); sys.exit()

            grounding = json.loads(self.blackboard.grnd_msg.data)
            
        return status
