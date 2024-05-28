import py_trees
import rospy
import json
import std_msgs.msg as std_msgs

from py_trees_ros import subscribers

class ToBlackboard(subscribers.ToBlackboard):

    def __init__(self, name, topic_name="symbol_grounding"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=std_msgs.String,
                                           blackboard_variables={"grnd_msg": None}, 
                                           clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE 
                                           )

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.stop_cmd = False


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
                    
            #self.feedback_message = "Grounding is ok"
            
        return status
