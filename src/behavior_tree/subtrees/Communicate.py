import copy, sys
import numpy as np
import json

import rospy
import py_trees
import PyKDL
import tf

from geometry_msgs.msg import Pose, Point, Quaternion
from complex_action_client import misc
from riro_srvs.srv import String_None, String_NoneRequest, String_Pose, String_PoseResponse, Delivery_Ticket, Delivery_TicketRequest


class Hiring(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, idx, action_goal):
        """
        ::action_goal:: dict, ex) {'robot':'spot', 'job':'load', 'source':'picking_station1', 'object':'box_s1', 'task_id':'spot_load_box_s1}
        """
        super(Hiring, self).__init__(name=name)
        self._idx = idx
        self._action_goal = action_goal
        self._request_help_channel = '/request_help'
        rospy.loginfo(f"[Subtree] Hiring. __init__ done")


    def setup(self, timeout):
        rospy.loginfo(f"[Subtree] Hiring. setup() called")
        self.blackboard = py_trees.Blackboard()
        self.feedback_message = "{}: setup".format(self.name)
        self.request_help_srv_req = rospy.ServiceProxy(self._request_help_channel, String_None)
        self.feedback_message = "{}: finished setting up".format(self.name)
        return True


    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        self._task_name = self._action_goal['task_id']

    def update(self):
        rospy.loginfo(f"[Subtree] Hiring. update() called")
        self.logger.debug("%s.update()" % self.__class__.__name__)
        
        if not self.sent_goal:
            rospy.loginfo(f"[Subtree] Hiring. action_goal:{self._action_goal}")
            rospy.wait_for_service(self._request_help_channel)
            rospy.loginfo(f"[Subtree] Hiring. service waited done")
            req = String_NoneRequest()
            req.data = json.dumps(self._action_goal)
            self.request_help_srv_req(req)
            self.sent_goal = True
            rospy.loginfo(f"Triying to find someone to help '{self._action_goal['robot']}' ({self._action_goal['job']})")
            self.feedback_message = "Trying to find someone to help me."
            return py_trees.common.Status.SUCCESS
        
        return py_trees.common.Status.SUCCESS

    
    def terminate(self, new_status):
        return

class Assigning(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, idx, action_goal):
        """
        ::action_goal:: dict, ex) {'robot':'spot', 'task_id':'spot_load_box_s1}
        """
        super(Assigning, self).__init__(name=name)
        self._idx = idx
        self._action_goal = action_goal
        self._task_assigning_channel = '/assign_task'
        self._task_id = None

    def setup(self, timeout):
        rospy.loginfo("[Subtree] Assigning : setup() called.")
        self.blackboard = py_trees.Blackboard()
        self.task_assign_srv_req = rospy.ServiceProxy(self._task_assigning_channel, String_None)
        self.feedback_message = "{}: finished setting up".format(self.name)
        rospy.loginfo("[Subtree] Assigning : setup() done.")
        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        self._task_id = self._action_goal['task_id']

    def update(self):
        if self._action_goal['robot'] is None:
            return py_trees.common.Status.FAILURE
        rospy.loginfo("[Subtree] Assigning : update() called.")
        req = String_NoneRequest()
        req.data = json.dumps({'admin':self._action_goal['robot'], 'task_id':self._action_goal['task_id']})
        self.task_assign_srv_req(req)
        self.sent_goal = True
        rospy.loginfo(f"Task assign is requested ({self._action_goal['robot']}).")
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return


class Checking(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, idx, action_goal):
        """
        ::action_goal:: dict, ex) {'robot':'spot', 'task_id':'spot_load_box_s1}
        """
        super(Checking, self).__init__(name=name)
        self._idx = idx
        self._action_goal = action_goal
        self._task_confirm_channel = '/confirm_result'

    def setup(self, timeout):
        rospy.loginfo("[Subtree] Checking : setup() called.")
        self.blackboard = py_trees.Blackboard()
        self.feedback_message = "{}: setup".format(self.name)
        self.task_confirm_srv_req = rospy.ServiceProxy(self._task_confirm_channel, String_None)
        self.feedback_message = "{}: finished setting up".format(self.name)
        rospy.loginfo("[Subtree] Checking : setup() done.")
        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        self._task_name = self._action_goal['task_id']

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        task_list = self.blackboard.get('task_status') 
        task_list = json.loads(task_list.data) # Dict : {'TASK_NAME': {'admin':'spot', 'status':0}}
        rospy.loginfo(f"[Checking] update() called -- {task_list}")
        if self._task_name not in task_list.keys():
            rospy.loginfo(f"There is delay ({self._task_name}).")
            self.feedback_message = "I opened the submission box right before."
            return py_trees.common.Status.RUNNING
        result = task_list[self._task_name]['status']
        if result == 0:
            rospy.loginfo(f"Not finished yet ({self._task_name}).")
            self.feedback_message = "Not finished yet"
            return py_trees.common.Status.RUNNING
        elif result == 1:
            req = json.dumps({'admin':self._action_goal['robot'], 'task_id':self._action_goal['task_id']})
            self.task_confirm_srv_req(req)
            rospy.loginfo(f"Finished requested task ({self._task_name}).")
            self.feedback_message = "Good"
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return

class Submit(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, idx, action_goal):
        """
        ::action_goal:: dict, ex) {'status':1, 'task_id':'spot_load_box_s1}
        """
        super(Submit, self).__init__(name=name)
        self._idx = idx
        self._action_goal = action_goal
        self._task_submission_channel = '/submit_result'

    def setup(self, timeout):
        rospy.loginfo("[Subtree] SUBMIT : setup() called.")
        self.blackboard = py_trees.Blackboard()
        self.feedback_message = "{}: setup".format(self.name)
        self.task_submission_srv_req = rospy.ServiceProxy(self._task_submission_channel, String_None)
        self.feedback_message = "{}: finished setting up".format(self.name)
        rospy.loginfo("[Subtree] SUBMIT : setup() done.")
        return True

    def initialise(self):
        self.feedback_message = "Initialise"
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))
        self.sent_goal = False
        self._task_name = self._action_goal['task_id']

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        task_list = self.blackboard.get('task_status') 
        task_list = json.loads(task_list.data) # Dict : {'TASK_NAME': {'admin':'spot', 'status':0}}
        
        if self._task_name not in task_list.keys():
            rospy.loginfo(f"There is no task to submit ({self._task_name}).")
            self.feedback_message = "Task Not Assigned yet"
            return py_trees.common.Status.RUNNING
        
        result = task_list[self._task_name]['status']
        if result == 1:
            rospy.loginfo(f"Someone submitted already ({self._task_name}).")
            self.feedback_message = "Someone submitted already"
            return py_trees.common.Status.FAILURE
        elif result == 0:
            req = json.dumps({'task_id':self._task_name, 'status':1})
            self.task_submission_srv_req(req)
            rospy.loginfo(f"Assignment Submission ({self._task_name}).")
            self.feedback_message = "Good"
            return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        return


class SpotLeave(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, unload=None):
        super(SpotLeave, self).__init__(name=name)
        if unload is None:
            self.unload = None
            self.is_unload = False
        else:
            self.unload = unload
            self.is_unload = True

        self._update_load_state_srv_channel = "/update_load_state"


    def setup(self, timeout):
        self.feedback_message = "{}: setup".format(self.name)
        rospy.wait_for_service(self._update_load_state_srv_channel, rospy.Duration(3))
        self.update_load_req = rospy.ServiceProxy(self._update_load_state_srv_channel, String_None)
        return True


    def initialise(self):
        self.logger.debug("{0}.initialise()".format(self.__class__.__name__))

    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)
        self.update_load_req(self.unload)
        return py_trees.common.Status.SUCCESS
            
    
    def terminate(self, new_status):
        return
