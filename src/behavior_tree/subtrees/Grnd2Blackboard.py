# Standard imports
import json
import queue

# Third-party imports
import py_trees
from py_trees_ros import subscribers

# ROS imports
import rospy
import std_msgs.msg as std_msgs

class ToBlackboard(subscribers.ToBlackboard):

    def __init__(self, name, topic_name="symbol_grounding"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=std_msgs.String,
                                           blackboard_variables={"grnd_msg": None}, 
                                           clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE 
                                           )

        # Blackboard initialization
        self.blackboard = py_trees.blackboard.Blackboard()

        # stop command
        self.blackboard.stop_cmd = False

        # Waypoint navigtation
        # TODO: Implement world model for to replace goal_loc_dict
        self.blackboard.goal_num = 0 # To track the number of goals
        self.blackboard.list_goals = [] # To store and lookup the goals

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        self.blackboard.stop_cmd = False
        
        if status != py_trees.common.Status.RUNNING:

            # we got something
            if self.blackboard.grnd_msg.data is None:
                rospy.logwarn_throttle(60, "%s: No grounding on the blackboard!" % self.name)
            grounding = json.loads(self.blackboard.grnd_msg.data)
            
            # container for commands
            ## self.blackboard.groundings        = []
            
            for param_id in range(grounding['param_num']):
                primitive_action = grounding['params'][str(param_id+1)]['primitive_action']
                
                if primitive_action == "stop":
                    self.blackboard.stop_cmd = True
                    break
                elif primitive_action == "move_to_goal":
                    self.blackboard.goal_num += 1
                    self.blackboard.list_goals.append(grounding['params'][str(param_id+1)])
                    break
                else:
                    rospy.logwarn_throttle(60, "%s: Unknown primitive action!" % self.name)
                    self.blackboard.stop_cmd = True
                    break
            
        return status
