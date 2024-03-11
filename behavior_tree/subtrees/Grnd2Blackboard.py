import py_trees
import py_trees_ros
import rclpy
import json
import std_msgs.msg as std_msgs

from py_trees_ros import subscribers

class ToBlackboard(subscribers.ToBlackboard):

    def __init__(self,
                 name: str,
                 topic_name: str):
        super(ToBlackboard, self).__init__(name=name,
                                           topic_name=topic_name,
                                           topic_type=std_msgs.String,
                                           blackboard_variables={"grnd_msg": None},
                                           clearing_policy=py_trees.common.ClearingPolicy.ON_INITIALISE,
                                           qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
                                           )

        self.blackboard.register_key(key="stop_cmd", access=py_trees.common.Access.WRITE)        
        self.blackboard.register_key(key="grnd_msg", access=py_trees.common.Access.READ)        
        self.blackboard.stop_cmd = False

        self.blackboard.register_key(key="loader_lib", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="loader_lib", access=py_trees.common.Access.READ)
        loader_lib = {}
        self.blackboard.set('loader_lib', loader_lib)


    def update(self) -> py_trees.common.Status:
        """
        Call the parent to write the raw data to the blackboard.

        Returns:
            :attr:`~py_trees.common.Status.SUCCESS` if a message was written, :attr:`~py_trees.common.Status.RUNNING` otherwise.
        """        
        self.logger.debug("%s.update()" % self.__class__.__name__)
        status = super(ToBlackboard, self).update()
        self.blackboard.stop_cmd = False
        
        if status != py_trees.common.Status.RUNNING:

            # we got something
            if self.blackboard.grnd_msg.data is None:
                self.node.get_logger().warning("%s: No grounding on the blackboard!" % self.name)
            grounding = json.loads(self.blackboard.grnd_msg.data)
            
            for param_id in range(grounding['param_num']):
                primitive_action = grounding['params'][str(param_id+1)]['primitive_action'].encode('ascii','ignore')
                
                if primitive_action == "stop":
                    self.blackboard.stop_cmd = True
                    break
                    
        return status
