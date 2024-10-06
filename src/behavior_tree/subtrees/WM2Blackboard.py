import py_trees
import rospy
import std_msgs.msg as std_msgs
import json

from py_trees_ros import subscribers

class ToBlackboard(subscribers.ToBlackboard):

    def __init__(self, name, topic_name="/world_model"):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=std_msgs.String,
                                           blackboard_variables={"wm_msg": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER
                                           )

        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.wm_dict = {}

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        if status != py_trees.common.Status.RUNNING:
            # we got something
            if self.blackboard.wm_msg.data is None:
                self.blackboard.no_wm_warning=True
                rospy.logwarn_throttle(60, "%s: No world model on the blackboard!" % self.name)
            else:
                self.blackboard.no_wm_warning=False
                
                # Save the dictionary sent by the 'wm_msg' topic
                self.blackboard.wm_msg = json.loads(self.blackboard.wm_msg.data)
                
                for i in self.blackboard.wm_msg['param_num']:
                    self.blackboard.wm_dict[str[i + 1]] = \
                        self.blackboard.wm_msg['worldmodel_dict'][str[i + 1]]
                
            self.feedback_message = "No world model " if self.blackboard.no_wm_warning \
                                                      else "World model is ok"
            
        return status
