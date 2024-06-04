# Standard imports
import json
import queue

# Third-party imports
import py_trees
from py_trees_ros import subscribers

# ROS imports
import rospy
import std_msgs.msg as std_msgs
from actionlib_msgs.msg import GoalStatus

# Local import
from riro_navigation.msg import TaskPlanAction, TaskPlanResult, TaskPlanFeedback, GoalInfo, GoalsInfo
from riro_navigation.msg import Dictlistfloat
from riro_navigation.srv import LTLPlan, LTLPlanRequest

class ToBlackboard(subscribers.ToBlackboard):

    def __init__(self, name, topic_name="/task_goals"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name = topic_name,
                                           topic_type = std_msgs.String,
                                           blackboard_variables = {"goals": None},
                                           clearing_policy = py_trees.common.ClearingPolicy.NEVER
                                           )

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.goal_num = 0 # To track the number of goals
        self.blackboard.list_goals = [] # To store and lookup the goals
        self.blackboard.queue_goals = queue.Queue() # To track the progress

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()

        # We've got new goals
        if status != py_trees.common.Status.RUNNING:
            # 'status' isn't RUNNING, but no goals are on the blackboard
            if self.blackboard.grnd_msg.data is None:
                rospy.logwarn_throttle(60, "%s: No goals on the blackboard!" % self.name)
            
            # Load the goals from the blackboard
            goals = json.loads(self.blackboard.goals.data)
        
            # Create queue for goals  
            for goal in goals:
                self.blackboard.goal_num += 1
                self.blackboard.list_goals.append(goal)
                self.blackboard.queue_goals.put(goal)
                rospy.loginfo("%s: Goal %d: added" % (self.__class__.__name__, 
                                                      self.blackboard.goal_num
                                                      ))
