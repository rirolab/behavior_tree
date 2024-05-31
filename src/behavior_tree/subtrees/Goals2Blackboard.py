# Standard imports
import json

# Third-party imports
import py_trees
from py_trees_ros import subscribers

# ROS imports
import rospy
import actionlib
import std_msgs.msg as std_msgs
from actionlib_msgs.msg import GoalStatus

# Local imports
from riro_navigation.msg import TaskPlanAction, TaskPlanResult, TaskPlanFeedback
from riro_navigation.msg import Dictlistfloat
from riro_navigation.srv import LTLPlan, LTLPlanRequest

class Goals2Blackboard:
    def __init__(self):
        
        # Action server to receive task plans
        self.task_plan_server = actionlib.SimpleActionServer('task_plan', TaskPlanAction, self.execute_task_plan, False)
        self.task_plan_server.start()
        
        # Blackboard
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.queue = []

    def execute_task_plan(self, goal):
        # Add goal to the blackboard queue
        self.blackboard.queue.append((goal.goal_x, goal.goal_y, goal.final))

        # Add new goal behavior to the sequence
        goal_behavior = self.create_goal_behavior(goal.goal_x, goal.goal_y, goal.final)
        self.sequence.add_child(goal_behavior)

        # Provide feedback
        feedback = TaskPlanFeedback()
        feedback.relaxation_required = False
        self.task_plan_server.publish_feedback(feedback)

        # Simulate goal execution and send result
        self.execute_goals()

    def create_goal_behavior(self, goal_x, goal_y, final):
        return py_trees.behaviours.Success(name=f"Go to ({goal_x}, {goal_y})")

    def execute_goals(self):
        while self.blackboard.queue:
            goal_x, goal_y, final = self.blackboard.queue.pop(0)
            rospy.loginfo(f"Executing goal: ({goal_x}, {goal_y}), final: {final}")

            # Simulate navigation to the goal (here you would implement the real navigation logic)
            rospy.sleep(2)  # Simulate time to navigate

            # Send result
            result = TaskPlanResult()
            result.relaxation = final
            result.robot_x = goal_x  # Simulate robot's current position
            result.robot_y = goal_y  # Simulate robot's current position
            result.goal_x = goal_x
            result.goal_y = goal_y
            self.task_plan_server.set_succeeded(result)
            rospy.loginfo(f"Goal ({goal_x}, {goal_y}) reached, final: {final}")

if __name__ == '__main__':
    try:
        Goals2Blackboard()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

# # Get the 'goal' from the 'task_plan' and save it as a queue in the blakcboard
# class ToBlackboard(subscribers.ToBlackboard):
#     def __init__(self, name, topic_name="task_plan"):
#         super(ToBlackboard, self).__init__(name=name,
#                                            topic_name=topic_name,
#                                            topic_type=std_msgs.String,
#                                            blackboard_variables={"task_plan": None},
#                                            clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE
#                                            )

#         self.blackboard = py_trees.blackboard.Blackboard()
#         self.blackboard.stop_cmd = False
#         self.blackboard.task_plan = None

#         # Action server client to listen to the task plan
#         self.task_plan_server = actionlib.SimpleActionServer('task_plan', TaskPlanAction, 
#                                                              self.execute_task_plan, False)
#         self.task_plan_server.start()


    
#     # Check if the task plan is available
#     def update(self):
#         self.logger.debug("%s.update()" % self.__class__.__name__)
#         status = super(ToBlackboard, self).update()
#         self.blackboard.stop_cmd = False
        
#         if status != py_trees.common.Status.RUNNING:
#             # we got something
#             if self.blackboard.task_plan.data is None:
#                 rospy.logwarn_throttle(60, "%s: No task plan on the blackboard!" % self.name)
#             else:
#                 self.blackboard.no_task_plan_warning=False
                
#             self.feedback_message = "No task plan " if self.blackboard.no_task_plan_warning else "Task plan is ok"
            
#         return status