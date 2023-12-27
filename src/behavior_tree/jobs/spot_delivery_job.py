
import copy, sys
import move_base_msgs.msg as move_base_msgs
import py_trees, py_trees_ros
import rospy
import threading
import numpy as np
import json
import PyKDL
import tf

import std_msgs.msg as std_msgs
from complex_action_client import misc

#sys.path.insert(0,'..')
from behavior_tree.subtrees import MoveJoint, MovePose, MoveBase, Gripper, Stop, WorldModel, Communicate
from behavior_tree.decorators import Ticketing, Replanning

##############################################################################
# Behaviours
##############################################################################


class Move(object):
    """
    A job handler that instantiates a subtree for scanning to be executed by
    a behaviour tree.
    """

    def __init__(self):
        """
        Tune into a channel for incoming goal requests. This is a simple
        subscriber here but more typically would be a service or action interface.
        """
        self._grounding_channel = "symbol_grounding" #rospy.get_param('grounding_channel')
        self._name = "delivery"
        ## self._subscriber = rospy.Subscriber("/dashboard/move", std_msgs.Empty, self.incoming)
        # self._subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
        self._goal = None
        self._lock = threading.Lock()
        
        self.blackboard = py_trees.blackboard.Blackboard()

    @property
    def name(self):
        return self._name
    @name.getter
    def name(self):
        return self._name    
    @property
    def goal(self):
        """
        Getter for the variable indicating whether or not a goal has recently been received
        but not yet handled. It simply makes sure it is wrapped with the appropriate locking.
        """
        with self._lock:
            g = copy.copy(self._goal) or self._goal
        return g

    @goal.setter
    def goal(self, value):
        """
        Setter for the variable indicating whether or not a goal has recently been received
        but not yet handled. It simply makes sure it is wrapped with the appropriate locking.
        """
        with self._lock:
            self._goal = value

    def incoming(self, msg):
        """
        Incoming goal callback.

        Args:
            msg (:class:`~std_msgs.Empty`): incoming goal message
        """
        if self.goal:
            rospy.logerr("MOVE: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range( len(list(grounding.keys())) ):
                if grounding[str(i+1)]['primitive_action'] in ['delivery']:
                    rospy.loginfo("[Job] Delivery arrived.")
                    self.goal = grounding #[str(i+1)] )
                    break
                
            
    ## def create_report_string(self, subtree_root):
    ##     """
    ##     Introspect the subtree root to determine an appropriate human readable status report string.

    ##     Args:
    ##         subtree_root (:class:`~py_trees.behaviour.Behaviour`): introspect the subtree

    ##     Returns:
    ##         :obj:`str`: human readable substring
    ##     """
    ##     if subtree_root.tip().has_parent_with_name("Cancelling?"):
    ##         return "cancelling"
    ##     else:
    ##         return "scanning"


    @staticmethod
    def create_root(idx="1", goal=std_msgs.Empty(), controller_ns="", **kwargs):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        # beahviors
        root = py_trees.composites.Sequence(name="Delivery")
        blackboard = py_trees.blackboard.Blackboard()
        
        if goal[idx]["primitive_action"] in ['delivery']:
            robot       = goal[idx]['robot']
            obj         = goal[idx]['object']
            source      = goal[idx]['source']
            destination = goal[idx]['destination']
        else:
            return None

        rospy.loginfo(f"spot_delivery_job (create root) start")
        # ----------------- Bring ---------------------
        bring = py_trees.composites.Sequence(name="Bring")
        pose_est10 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot':robot,'destination': source})
        ticketing1 = Ticketing(child=pose_est10, idx=idx, name="Ticketing")
        s_drive10 = MoveBase.MOVEB(name="Drive", idx=idx, destination=source,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"})
        
        replanning1 = Replanning(s_drive10, idx=idx, name="Replan")
        
        task_id = f"{robot}_load_{obj}"
        hiring1 = Communicate.Hiring(name="Hiring", idx=idx, 
                                     action_goal={'robot':robot, 'job':'help_load', 'object':obj, 'source':source, 'destination':f'{robot}_table_from_gz', 'task_id':task_id})
        assign11 = Communicate.Assigning(name="Assignment_come", idx=idx, action_goal={'robot':robot, 'task_id':f'{task_id}_come'})
        assign12 = Communicate.Assigning(name="Assignment_load", idx=idx, action_goal={'robot':robot, 'task_id':f'{task_id}_load'})
        ask_help1 = py_trees.composites.Sequence(name="AskingHelp", children=[hiring1, assign11, assign12])
        run_once1 = py_trees.decorators.OneShot(name="Once", child=ask_help1)
        
        waiting1 = py_trees.composites.Parallel(name='Waiting', children=[ticketing1, replanning1, run_once1])
        approaching1 = MoveBase.TOUCHB(name="Touch", idx=idx, destination=source, 
                                      action_goal={'pose': "Plan"+idx+"/parking_pose"})
        arrived1 = Communicate.Submit(name="Arrived", idx=idx, action_goal={'task_id':f'{task_id}_arrived', 'status':1})
        checking1 = Communicate.Checking(name="Come?", idx=idx, action_goal={'robot':robot, 'task_id':f'{task_id}_come'})
        bring.add_children([waiting1, approaching1, arrived1, checking1])

        # ----------------- Load ---------------------
        load2 = py_trees.composites.Sequence(name="Load")
        checking2 = Communicate.Checking(name="Loaded?", idx=idx, action_goal={'robot':robot, 'task_id':f'{task_id}_load'})
        load2.add_children([checking2])

        # ----------------- Delivery ---------------------
        delivery = py_trees.composites.Sequence(name="Delivery")
        pose_est30 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot':robot,'destination': destination})
        ticketing3 = Ticketing(child=pose_est30, idx=idx, name="Ticketing")
        s_drive30 = MoveBase.MOVEB(name="Drive", idx=idx, destination=destination,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"})
        replanning3 = Replanning(s_drive30, idx=idx, name="Replan")
        
        task_id = f"{robot}_unload_{obj}"
        hiring3 = Communicate.Hiring(name="Hiring", idx=idx, 
                                     action_goal={'robot':robot, 'job':'help_unload', 'object':obj, 'source':f'{robot}_table_from_gz', 'destination':destination, 'task_id':task_id})
        assign31 = Communicate.Assigning(name="Assignment_come", idx=idx, action_goal={'robot':robot, 'task_id':f'{task_id}_come'})
        assign32 = Communicate.Assigning(name="Assignment_unload", idx=idx, action_goal={'robot':robot, 'task_id':f'{task_id}_unload'})
        ask_help3 = py_trees.composites.Sequence(name="AskingHelp", children=[hiring3, assign31, assign32])
        run_once3 = py_trees.decorators.OneShot(name="Once", child=ask_help3)
        
        waiting3 = py_trees.composites.Parallel(name='Waiting', children=[ticketing3, replanning3, run_once3])
        approaching3 = MoveBase.TOUCHB(name="Touch", idx=idx, destination=destination,
                                      action_goal={'pose': "Plan"+idx+"/parking_pose"})
        arrived3 = Communicate.Submit(name="Arrived", idx=idx, action_goal={'task_id':f'{task_id}_arrived', 'status':1})
        checking3 = Communicate.Checking(name="Come?", idx=idx, action_goal={'robot':robot, 'task_id':f'{task_id}_come'})
        delivery.add_children([waiting3, approaching3, arrived3, checking3])

        # ----------------- Load ---------------------
        unload = py_trees.composites.Sequence(name="Unload")
        checking4 = Communicate.Checking(name="Unloaded?", idx=idx, action_goal={'robot':robot, 'task_id':f'{task_id}_unload'})
        unload.add_children([checking4])
        
        task = py_trees.composites.Sequence(name="Delivery")
        task.add_children([bring, load2, delivery, unload])
        
        return task


