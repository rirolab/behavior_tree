# Standard imports
import json

# Third-party imports
import py_trees
from py_trees_ros import subscribers

# ROS imports
import rospy
import actionlib
import std_msgs.msg as std_msgs

# Local imports
from riro_navigation.msg import TaskPlanAction, TaskPlanResult
from riro_navigation.msg import Dictlistfloat
from riro_navigation.srv import LTLPlan, LTLPlanRequest

# Get the 'goal' from the 'task_plan' and save it as a queue in the blakcboard
class ToBlackboard(subscribers.ToBlackboard):
    def __init__(self, name, topic_name="task_plan"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=std_msgs.String,
                                           blackboard_variables={"task_plan": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
                                           )

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.stop_cmd = False
        self.blackboard.task_plan = None

        # Action server client to listen to the task plan
        self.task_plan_server = actionlib.SimpleActionServer('task_plan', TaskPlanAction, 
                                                             self.execute_task_plan, False)
        self.task_plan_server.start()


    
    # Check if the task plan is available
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        self.blackboard.stop_cmd = False
        
        if status != py_trees.common.Status.RUNNING:
            # we got something
            if self.blackboard.task_plan.data is None:
                rospy.logwarn_throttle(60, "%s: No task plan on the blackboard!" % self.name)
            else:
                self.blackboard.no_task_plan_warning=False
                
            self.feedback_message = "No task plan " if self.blackboard.no_task_plan_warning else "Task plan is ok"
            
        return status