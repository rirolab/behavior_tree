import py_trees
import rospy
import std_msgs.msg as std_msgs

from py_trees_ros import subscribers

class ToBlackboard(subscribers.ToBlackboard):
    '''Task Status to Blackboard'''
    def __init__(self, name, topic_name="/task_status"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=std_msgs.String,
                                           blackboard_variables={"task_status": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )

        self.blackboard = py_trees.blackboard.Blackboard()
        

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something
            if self.blackboard.task_status.data is None:
                self.blackboard.no_task_status_warning=True
                rospy.logwarn_throttle(60, "%s: No TASK STATUS on the blackboard!" % self.name)
            else:
                self.blackboard.no_task_status_warning=False
                
            self.feedback_message = "No Task Status " if self.blackboard.no_task_status_warning else "Task Status is ok"
            
        return status
