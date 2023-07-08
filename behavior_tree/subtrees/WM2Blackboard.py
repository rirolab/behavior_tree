import py_trees
import py_trees_ros
import rclpy
import std_msgs.msg as std_msgs

from py_trees_ros import subscribers

class ToBlackboard(subscribers.ToBlackboard):

    def __init__(self,
                 name: str,
                 topic_name: str,
                 qos_profile: rclpy.qos.QoSProfile):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=std_msgs.String,
                                           blackboard_variables={"wm_msg": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.NEVER,
                                           qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                           )

        #self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.register_key(key="no_wm_warning", access=py_trees.common.Access.WRITE)

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
            if self.blackboard.wm_msg.data is None:
                self.blackboard.no_wm_warning=True
                self.node.get_logger().warning("%s: No world model on the blackboard!" % self.name)
            else:
                self.blackboard.no_wm_warning=False
                
            self.feedback_message = "No world model " if self.blackboard.no_wm_warning else "World model is ok"
            
        return status
