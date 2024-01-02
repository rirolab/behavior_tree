import time
import py_trees
from py_trees.decorators import *
import rospy
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from dynamic_reconfigure.msg import DoubleParameter

class Loop(Decorator):
    """
    A decorator that loops the node or branch a number of times, or infinitely.
    """
    def __init__(self, child, name=common.Name.AUTO_GENERATED, n_loop=100, enable_inf_loop=False,
                 timeout=-1, condition=None, return_success=False):
        """
        Init with the decorated child.
        
        Args:
        child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
        name (:obj:`str`): the decorator name
        n_loop: the number of loops to run
        enable_inf_loop: run indefinitely
        timeout: timeout value if enalble_inf_loop is True (a negative timeout will loop forever).
        """
        super(Loop, self).__init__(name=name, child=child)

        self.n_loop          = n_loop
        self.enable_inf_loop = enable_inf_loop
        self.duration        = timeout
        self.condition       = condition
        self.finish_time     = None
        self.return_success  = return_success

    def initialise(self):
        """
        Reset the feedback message and finish time on behaviour entry.
        """
        self.finish_time = time.time() + self.duration
        self.feedback_message = ""
        self.count = 0
        
    def update(self):
        """
        Flip :data:`~py_trees.common.Status.FAILURE` and
        :data:`~py_trees.common.Status.SUCCESS`
        
        Returns:
        :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """

        current_time = time.time()
        if self.enable_inf_loop and self.duration >= 0 and current_time > self.finish_time:
            self.feedback_message = "timed out"
            self.logger.debug("{}.update() {}".format(self.__class__.__name__, self.feedback_message))
            # invalidate the decorated (i.e. cancel it), could also put this logic in a terminate() method
            self.decorated.stop(common.Status.INVALID)
            return common.Status.FAILURE

        if self.decorated.status == common.Status.SUCCESS:
            self.count += 1
            if self.condition is not None and self.decorated.status == self.condition:
                return common.Status.SUCCESS                
            if self.enable_inf_loop is False and self.count > self.n_loop:
                return common.Status.SUCCESS
            
            if self.enable_inf_loop:
                self.feedback_message = self.decorated.feedback_message + " [Infinite loop]"
            else:
                self.feedback_message = self.decorated.feedback_message + " [n_loop: {}/{}]".format(self.count, self.n_loop)
            return common.Status.RUNNING

        if self.enable_inf_loop:
            self.feedback_message = self.decorated.feedback_message + " [Infinite loop]"
            return common.Status.RUNNING
        else:
            if self.condition is not None and self.decorated.status != self.condition:
                self.feedback_message = self.decorated.feedback_message + " [loop under condition]"
                return common.Status.RUNNING
                            
            self.feedback_message = self.decorated.feedback_message
            return self.decorated.status
    
class Ticketing(Decorator):
    """
    A decorator that loops the node or branch a number of times, or infinitely.
    """
    def __init__(self, child, idx, name=common.Name.AUTO_GENERATED):
        """
        Init with the decorated child.
        
        Args:
        child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
        name (:obj:`str`): the decorator name
        """
        super(Ticketing, self).__init__(name=name, child=child)
        self.idx = idx
        rospy.loginfo("(Ticketing) decorator initialized.")

    def setup(self,timeout):
        rospy.loginfo("(Ticketing) decorator setup function called.")
        return super(Ticketing, self).setup(timeout)
    
    def initialise(self):
        """
        Reset the feedback message and finish time on behaviour entry.
        """
        self.blackboard = py_trees.Blackboard()
        self.blackboard.set('Plan'+self.idx+'/ticket', -1)
        self.feedback_message = "[(Decorator) Heading] Heading to somewhere."
        rospy.loginfo("(Ticketing) decorator initialize function called.")
        
    def update(self):
        """
        Flip :data:`~py_trees.common.Status.FAILURE` and
        :data:`~py_trees.common.Status.SUCCESS`
        
        Returns:
        :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        rospy.loginfo(f'[Ticketing] updated called')
        order = self.blackboard.get('Plan'+self.idx+'/ticket')
        if order == 0:
            self.feedback_message = f'(Ticketing) current order: {order}'
            rospy.loginfo(f"[Ticketing] SUCCESS order={order}")
            return common.Status.SUCCESS
        rospy.loginfo(f"[Ticketing] running order={order}")
        return common.Status.RUNNING

class Replanning(Decorator):
    """
    A decorator that loops the node or branch a number of times, or infinitely.
    """
    def __init__(self, child, idx, name=common.Name.AUTO_GENERATED):
        """
        Init with the decorated child.
        
        Args:
        child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
        name (:obj:`str`): the decorator name
        """
        super(Replanning, self).__init__(name=name, child=child)
        self.idx = idx
        rospy.loginfo("(Replanning) decorator initialized.")

    def setup(self,timeout):
        rospy.loginfo("[Decorator] Replanning: setup() done.")
        return super(Replanning, self).setup(timeout)
    
    def initialise(self):
        """
        Reset the feedback message and finish time on behaviour entry.
        """
        self.ticket = -1
        self.sent_goal = False
        self.blackboard = py_trees.Blackboard()
        self.feedback_message = "[(Decorator) Replanning] Heading to somewhere."
        rospy.loginfo("(Replanning) decorator initialize function called.")
        
    def update(self):
        """
        Flip :data:`~py_trees.common.Status.FAILURE` and
        :data:`~py_trees.common.Status.SUCCESS`
        
        Returns:
        :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        rospy.loginfo(f'[Replannig] updated called')
        prev_ticket = self.ticket
        self.ticket = self.blackboard.get('Plan'+self.idx+'/ticket')
        if not self.sent_goal:
            rospy.loginfo(f"[Replanning] moving to somewhere ticket={self.ticket}")
            self.decorated.initialise()
            self.sent_goal = True
            return common.Status.RUNNING
        
        if self.decorated.status == common.Status.FAILURE:
            rospy.loginfo(f'[Replannig] child is failed')
            return common.Status.FAILURE
        
        if self.ticket != prev_ticket:
            self.feedback_message = f'[Replanning] ticket order changed {prev_ticket}->{self.ticket}'
            self.decorated.initialise()
            rospy.loginfo(f'[Replanning] ticket order changed {prev_ticket}->{self.ticket}')
            return common.Status.RUNNING
        elif self.ticket == 0:
            if self.decorated.status == common.Status.SUCCESS:
                rospy.loginfo(f'[Replanning] 1st order & move compledted.')
                return common.Status.SUCCESS
        
        rospy.loginfo(f"[Replanning] moving to somewhere ticket={self.ticket}")
        return common.Status.RUNNING


class Reconfiguring(Decorator):
    """
    A decorator that loops the node or branch a number of times, or infinitely.
    """
    def __init__(self, child, idx, action_goal, name=common.Name.AUTO_GENERATED):
        """
        Init with the decorated child.
        
        Args:
        child (:class:`~py_trees.behaviour.Behaviour`): behaviour to time
        action_goal (:dict:): reconfiguration_parameters # ex: {'srv_name':'move_base/TebLocalPlannerROS/set_parameters,
                                                                'params': {'move_base/TebLocalPlannerROS/xy_goal_tolerance' : 0.05, ...}}
        name (:obj:`str`): the decorator name
        """
        super(Reconfiguring, self).__init__(name=name, child=child)
        self.idx = idx
        self.action_goal = action_goal
        self.prev_prams = None
        self.sent_goal = False
        rospy.loginfo("(Reconfigure) decorator initialized.")

    def setup(self,timeout):
        rospy.loginfo("(Reconfigure) decorator setup function called.")
        self.reconfigure_srv_req = rospy.ServiceProxy(self.action_goal['srv_name'], Reconfigure)
        return super(Reconfiguring, self).setup(timeout)
    
    def initialise(self):
        """
        Reset the feedback message and finish time on behaviour entry.
        """
        self.blackboard = py_trees.Blackboard()
        self.feedback_message = "[(Decorator) Reconfigure]."
        if self.prev_prams is None:
            self.prev_prams = dict()
            for k in self.action_goal['params'].keys():
                self.prev_prams[k] = rospy.get_param(k)
        rospy.loginfo("(Reconfigure) decorator initialize function called.")
        
    def update(self):
        """
        Flip :data:`~py_trees.common.Status.FAILURE` and
        :data:`~py_trees.common.Status.SUCCESS`
        
        Returns:
        :class:`~py_trees.common.Status`: the behaviour's new status :class:`~py_trees.common.Status`
        """
        rospy.loginfo(f'[Reconfiguring] updated called')
        if not self.sent_goal:
            reconfigure_req = ReconfigureRequest()
            for k,v in self.action_goal['params'].items():
                reconfigure_req.config.doubles.append(DoubleParameter(k, v))
            self.reconfigure_srv_req(reconfigure_req)
            self.sent_goal = True
            return common.Status.RUNNING
        if self.decorated.status == common.Status.SUCCESS:
            return common.Status.SUCCESS
        return common.Status.RUNNING
    
    def terminate(self, new_status):
        reconfigure_req = ReconfigureRequest()
        for k,v in self.prev_prams.items():
            reconfigure_req.config.doubles.append(DoubleParameter(k, v))
        self.reconfigure_srv_req(reconfigure_req)
        return super().terminate(new_status)