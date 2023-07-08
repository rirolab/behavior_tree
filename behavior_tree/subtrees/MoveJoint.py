import numpy as np
import json
import rclpy

import py_trees
from action_msgs.msg import GoalStatus
from . import Move

from riro_srvs.srv import StringInt

class MOVEJ(Move.MOVE):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_client, action_goal=None,
                 topic_name="", controller_ns=""):
        super(MOVEJ, self).__init__(name=name,
                                   action_client=action_client,
                                   action_goal=action_goal,
                                   topic_name=topic_name,
                                   controller_ns=controller_ns)

        ## self.topic_name    = topic_name
        ## self.controller_ns = controller_ns
        ## self.arm           = None
        ## self.action_goal   = action_goal
        ## self.sent_goal     = False
        ## self.cmd_req       = None

        ## self.qos_profile = QoSProfile(
        ##     reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
        ##     history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        ##     depth=10
        ## )

        
    ## def setup(self, timeout):
    ##     self.feedback_message = "{}: setup".format(self.name)
    ##     self.cmd_req = self.create_client(StringInt, 'arm_client/command',
    ##                                       qos_profile=qos_profile)
    ##     while not self.cmd_req.wait_for_service(timeout_sec=1.0):
    ##         self.get_logger().info('command service not available, waiting again...')
        
    ##     self.create_subscription(GoalStatus, 'arm_client/goal_status',
    ##                                  self._goal_status_callback,
    ##                                  10)
    ##     return True


    ## def initialise(self):
    ##     self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
    ##     self.sent_goal = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            goal = {}
            for i, ang in enumerate(self.action_goal):
                goal[str(i)] = ang

            self.goal_uuid_des = np.random.randint(0, 255, size=16,
                                            dtype=np.uint8)
                
            cmd_str = json.dumps({'action_type': 'moveJoint',
                                  'goal': json.dumps(goal),
                                  'uuid': self.goal_uuid_des.tolist(),
                                  'timeout': 3.,
                                  'enable_wait': True})
            req = StringInt.Request()
            req.data = cmd_str
            
            self.future = self.cmd_req.call_async(req)
            ## ret = self.cmd_req(cmd_str)
            ## if ret.data==GoalStatus.STATUS_CANCELED or ret.data==GoalStatus.STATUS_ABORTED:
            ##     self.feedback_message = \
            ##       "failed to execute"
            ##     self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
            ##     return py_trees.common.Status.FAILURE
            
            self.sent_goal = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        ## msg = self.status_req()
        ## d = json.loads(msg.data)
        ## state = d['state']
        ## ret   = d['result']
        if self.goal_id is None:
            return py_trees.common.Status.RUNNING
            
        if (self.goal_uuid_des == self.goal_id.uuid).all() and \
           self.goal_status in [GoalStatus.STATUS_ABORTED,
                                GoalStatus.STATUS_UNKNOWN,
                                GoalStatus.STATUS_CANCELING,
                                GoalStatus.STATUS_CANCELED]:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.FAILURE, self.feedback_message))
            return py_trees.common.Status.FAILURE

        if (self.goal_uuid_des == self.goal_id.uuid).all() and \
           self.goal_status is GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.SUCCESS, self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
                

    ## def terminate(self, new_status):
    ##     msg = self.status_req()
    ##     d = json.loads(msg.data)
    ##     if d['state'] == GoalStatus.ACTIVE:
    ##         self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
    ##     self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))            
    ##     return 



class MOVEA(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None,
                 topic_name="", controller_ns=""):
        super(MOVEA, self).__init__(name=name)

        self.topic_name = topic_name
        self.controller_ns = controller_ns
        self.arm = None
        self.action_goal = action_goal
        self.sent_goal = False
        self.cmd_req   = None

    def setup(self, timeout):
        ## self.publisher = rospy.Publisher(self.topic_name, std_msgs.String, queue_size=10, latch=True)
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service("arm_client/command")
        self.cmd_req = rospy.ServiceProxy("arm_client/command", String_Int)
        rospy.wait_for_service("arm_client/status")
        self.status_req = rospy.ServiceProxy("arm_client/status", None_String)


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False


    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        if self.cmd_req is None:
            self.feedback_message = \
              "no action client, did you call setup() on your tree?"
            return py_trees.Status.FAILURE

        if not self.sent_goal:
            goal = {}
            for i, ang in enumerate(self.action_goal):
                goal[str(i)] = ang
            
            cmd_str = json.dumps({'action_type': 'moveJoint',
                                  'goal': json.dumps(goal),
                                  'timeout': 3.,
                                  'no_wait': True})
            ret = self.cmd_req(cmd_str)
            if ret.data==GoalStatus.REJECTED or ret.data==GoalStatus.ABORTED:
                self.feedback_message = \
                  "failed to execute"
                self.logger.debug("%s.update()[%s]" % (self.__class__.__name__, self.feedback_message))
                return py_trees.common.Status.FAILURE
            
            self.sent_goal = True
            self.feedback_message = "Sending a joint goal"
            return py_trees.common.Status.RUNNING

        msg = self.status_req()
        d = json.loads(msg.data)
        state = d['state']
        ret   = d['result']
        
        if  state in [GoalStatus.ABORTED,
                      GoalStatus.PREEMPTED,
                      GoalStatus.REJECTED] and \
                      ret != FollowJointTrajectoryResult.SUCCESSFUL:
            self.feedback_message = "FAILURE"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.FAILURE, self.feedback_message))
            return py_trees.common.Status.FAILURE

        if ret == FollowJointTrajectoryResult.SUCCESSFUL:
            self.feedback_message = "SUCCESSFUL"
            self.logger.debug("%s.update()[%s->%s][%s]" % (self.__class__.__name__, self.status, py_trees.common.Status.SUCCESS, self.feedback_message))
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.RUNNING
                

    def terminate(self, new_status):
        msg = self.status_req()
        d = json.loads(msg.data)
        if d['state'] == GoalStatus.ACTIVE:
            self.cmd_req( json.dumps({'action_type': 'cancel_goal'}) )
        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))            
        return 


    
