import typing
import json

import rclpy
## from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from action_msgs.msg import GoalStatus
from riro_srvs.srv import StringInt
## from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

import py_trees
from py_trees_ros import exceptions, utilities

class MOVE(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None,
                 topic_name="", controller_ns=""):
        super(MOVE, self).__init__(name=name)

        self.topic_name    = topic_name
        self.controller_ns = controller_ns
        self.arm           = None
        self.action_goal   = action_goal
        self.sent_goal     = False
        self.cmd_req       = action_client
        self.goal_uuid_des = None
        self.goal_id       = None
        self.goal_status   = None

        ## self.qos_profile = QoSProfile(
        ##     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        ##     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        ##     depth=10
        ## )
        ## self.callback_group = ReentrantCallbackGroup() #MutuallyExclusiveCallbackGroup
        
    def setup(self,
              node: typing.Optional[rclpy.node.Node]=None,
              node_name: str="tree",
              timeout: float=py_trees.common.Duration.INFINITE):
        """ """
        self.node = node
        self.feedback_message = "{}: setup".format(self.name)
        ## self.cmd_req = self.node.create_client(StringInt, 'arm_client/command',
        ##                                   qos_profile=self.qos_profile,
        ##                                   callback_group=self.callback_group)
        ## if not self.cmd_req.wait_for_service(timeout_sec=3.0):
        ##     raise exceptions.TimedOutError('command service not available, waiting again...')
        ##     #rclpy.spin_once(node, timeout_sec=0)
                    
        self.node.create_subscription(GoalStatus, 'arm_client/goal_status',
                                     self.goal_status_callback,
                                     10)


    def initialise(self):
        """ """
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        """ """
        return py_trees.common.Status.RUNNING

    
    def terminate(self, new_status):
        """ """
        if self.goal_id is None:
            self.node.get_logger().error("goal_id is not available")
            return
        
        if (self.goal_uuid_des == self.goal_id.uuid).all() and \
          self.goal_status == GoalStatus.STATUS_EXECUTING:
            req = StringInt.Request()
            req.data = json.dumps({'action_type': 'cancel_goal',
                                   'enable_wait': True})
            self.future = self.cmd_req.call_async( req )
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))            
        

    def goal_status_callback(self, msg):
        self.goal_id     = msg.goal_info.goal_id
        self.goal_status = msg.status
        
