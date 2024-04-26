import typing
import json

import rclpy
from action_msgs.msg import GoalStatus
from riro_srvs.srv import StringInt

import py_trees
from py_trees_ros import exceptions, utilities
import py_trees.console as console
## from rclpy.callback_groups import ReentrantCallbackGroup

class MOVE(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None, timeout=1.):
        super(MOVE, self).__init__(name=name)

        self.arm           = None
        self.action_goal   = action_goal
        self.sent_goal     = False
        self.cmd_req       = action_client
        self.goal_uuid_des = None
        ## self.goal_id       = None
        ## self.goal_status   = None
        self.timeout       = timeout

        self.blackboard = self.attach_blackboard_client(name=self.name)
        ## self.blackboard = py_trees.blackboard.Client()
        ## self.callback_group = ReentrantCallbackGroup() 
        self.blackboard.register_key(
            key="goal_id",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            key="goal_status",
            access=py_trees.common.Access.READ,
        )
        
    def setup(self, node):
        """ """
        self.feedback_message = "{}: setup".format(self.name)
        ## self.node = node
        ## self.node.create_subscription(GoalStatus, 'arm_client/goal_status',
        ##                              self.goal_status_callback,
        ##                              10,
        ##                              callback_group=self.callback_group)


    def initialise(self):
        """ """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        """ """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.sent_goal = True
        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        """ """
        self.logger.debug("%s.terminate()" % self.__class__.__name__)
        if self.blackboard.goal_id is None:
            self.feedback_message = "goal_id is not available"
            return py_trees.common.Status.SUCCESS
        
        self.logger.debug("self.goal_status".format(self.blackboard.goal_status))
        if (self.goal_uuid_des == self.blackboard.goal_id).all() and \
          self.blackboard.goal_status == GoalStatus.STATUS_EXECUTING:
            req = StringGoalStatus.Request()
            req.data = json.dumps({'action_type': 'cancel_goal',
                                   'enable_wait': True})
            self.future = self.cmd_req.call_async( req )
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))            
        return py_trees.common.Status.SUCCESS

    ## def goal_status_callback(self, msg):
    ##     console.loginfo("{}".format(str(msg)))
    ##     self.goal_id     = msg.goal_info.goal_id
    ##     self.goal_status = msg.status
        
