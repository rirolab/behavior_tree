
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
from behavior_tree.composites import PriorityParallel

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
        self._name = "help_unload"
        ## self._subscriber = rospy.Subscriber("/dashboard/move", std_msgs.Empty, self.incoming)
        # self._subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
        self._goal = None
        self._lock = threading.Lock()
        
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.gripper_open_pos = rospy.get_param("gripper_open_pos")
        self.blackboard.gripper_close_pos = rospy.get_param("gripper_close_pos")
        self.blackboard.gripper_open_force = rospy.get_param("gripper_open_force")
        self.blackboard.gripper_close_force = rospy.get_param("gripper_close_force")
        self.blackboard.init_config = eval(rospy.get_param("init_config", [0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.]))
        self.blackboard.drive_config = eval(rospy.get_param("drive_config", str([0, 0, 0, 0, 0, 0])))
        
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
                if grounding[str(i+1)]['primitive_action'] in ['help_unload']:
                    rospy.loginfo(f'[Job] Help Unload: HERE!!!!!!!!!!!!!!!!!!!!!!')
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
        rospy.loginfo("[Job] help : create_root() called.")
        root = py_trees.composites.Sequence(name="Help")
        blackboard = py_trees.blackboard.Blackboard()
        """ Ex) 
        [Load]                              |  [UnLoad]
        robot       : haetae,               |   robot       : haetae
        object      : box_s1                |   object      : box_s1
        source      : picking_station1      |   source      : spot_table
        destination : spot_table            |   destination : placing_shelf1
        task_id     : spot_load_box_s1      |   task_id     : spot_unload_box_s1
        """
        if goal[idx]["primitive_action"] in ['help_unload']:
            robot       = goal[idx]['robot']
            obj         = goal[idx]['object']
            source      = goal[idx]['source']
            destination = goal[idx]['destination']
            task_id     = goal[idx]['task_id']
        else:
            return None

        
        # ----------------- WAIT ---------------------
        wait = py_trees.composites.Sequence(name="Waiting")
        action_goal_arrived = {'robot':robot, 'task_id':f'{task_id}_arrived'}
        assignment1 = Communicate.Assigning(name="Arrived?", idx=idx, action_goal=action_goal_arrived)
        
        checking1 = Communicate.Checking(name="Checked", idx=idx, action_goal=action_goal_arrived)

        go_table = py_trees.composites.Sequence(name="GoTable")
        s_drive_pose1 = MoveJoint.MOVEJ(name="DrivePose", controller_ns=controller_ns,
                                  action_goal=blackboard.drive_config)
        pose_est1 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot':robot,'destination': destination, 'side':True})
        s_drive1 = MoveBase.MOVEB(name="Drive", idx=idx,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"}, destination=destination)
        go_table.add_children([s_drive_pose1, pose_est1, s_drive1])

        go_table_or_not = PriorityParallel(name="GoTable", children=[checking1, go_table])
        wait.add_children([assignment1, go_table_or_not])

        # ----------------- COME ---------------------
        come = py_trees.composites.Sequence(name="Come")
        pose_est2 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot':robot,'destination': source, 'side':True})
        s_drive2 = MoveBase.TOUCHB(name="Drive", idx=idx, destination=source,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"})
        submission2 = Communicate.Submit(name="Come", idx=idx, action_goal={'status':1, 'task_id':f'{task_id}_come'})

        come.add_children([pose_est2, s_drive2, submission2])
        rospy.loginfo("[Job] help : create_root() --- come done.")
        
        # ----------------- Pick ---------------------
        pose_est30 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj})
        s_init_pose30 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        s_move30 = MovePose.MOVEPROOT(name="Top1",
                                      controller_ns=controller_ns,
                                      action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move31 = MovePose.MOVEP(name="Top2", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move32 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                               action_goal=blackboard.gripper_open_pos,
                               force=blackboard.gripper_open_force)        
        s_move33 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_pose"})
        s_move34 = Gripper.GOTO(name="Close", controller_ns=controller_ns,
                               action_goal=blackboard.gripper_close_pos,
                               force=blackboard.gripper_close_force)        
        s_move35 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_init_pose31 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        
        pick = py_trees.composites.Sequence(name="Pick")
        pick.add_children([pose_est30, s_init_pose30, s_move30, s_move31, s_move32, s_move33, s_move34, s_move35, s_init_pose31])
        
        # ----------------- Approach ---------------------

        approach = py_trees.composites.Sequence(name="Approach")

        s_drive_pose4 = MoveJoint.MOVEJ(name="DrivePose", controller_ns=controller_ns,
                                  action_goal=blackboard.drive_config)
        pose_est4 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot':robot,'destination': destination, 'side':True})
        s_drive4 = MoveBase.TOUCHB(name="Drive", idx=idx, destination=destination,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"})
        submission4 = Communicate.Submit(name="Load", idx=idx, action_goal={'status':1, 'task_id':f'{task_id}_unload'})
        approach.add_children([s_drive_pose4, pose_est4, s_drive4, submission4])

        # ----------------- Load ---------------------

        place = py_trees.composites.Sequence(name="Place")
        s_init_pose50 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        s_init_pose51 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        pose_est5 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj,
                                                             'destination':destination})
        s_move50 = MovePose.MOVEPROOT(name="Top1",
                                      controller_ns=controller_ns,
                                      action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move51 = MovePose.MOVEP(name="Top2", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move52 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_pose"})
        s_move53 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force)        
        s_move54 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move55 = WorldModel.REMOVE(name="Remove", target=obj)
        
        place.add_children([pose_est5, s_init_pose50, s_move50, s_move51, s_move52, s_move53, s_move54, s_init_pose51, s_move55])
        # rospy.loginfo("[Job] help : create_root() --- place done.")

        task = py_trees.composites.Sequence(name="Help")
        # task.add_children([come, pick, approach, place])
        task.add_children([wait, come, pick, approach, place])
        return task


