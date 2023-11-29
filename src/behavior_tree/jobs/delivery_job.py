
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
from behavior_tree.subtrees import MoveJoint, MovePose, MoveBase, Gripper, Stop, WorldModel
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
        
        ## self._subscriber = rospy.Subscriber("/dashboard/move", std_msgs.Empty, self.incoming)
        self._subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
        self._goal = None
        self._lock = threading.Lock()
        
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.gripper_open_pos = rospy.get_param("gripper_open_pos")
        self.blackboard.gripper_close_pos = rospy.get_param("gripper_close_pos")
        self.blackboard.gripper_open_force = rospy.get_param("gripper_open_force")
        self.blackboard.gripper_close_force = rospy.get_param("gripper_close_force")
        self.blackboard.init_config = eval(rospy.get_param("init_config", [0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.]))
        self.blackboard.drive_config = [np.pi/2, -2.4, 2.4, -np.pi/2., -np.pi/2., 0]

        ## self.object      = None
        ## self.destination = None

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

        
        # ----------------- Bring ---------------------
        bring = py_trees.composites.Sequence(name="Bring")
        s_drive_pose1 = MoveJoint.MOVEJ(name="DrivePose", controller_ns=controller_ns,
                                  action_goal=blackboard.drive_config)
        pose_est10 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot':robot,'destination': source})
        ticketing1 = Ticketing(child=pose_est10, idx=idx, name="Ticketing")
        s_drive10 = MoveBase.MOVEB(name="Drive", idx=idx, destination=source,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"})
        replanning1 = Replanning(s_drive10, idx=idx, name="Replan")
        waiting1 = py_trees.composites.Parallel(name='Waiting', children=[ticketing1, replanning1])
        bring.add_children([s_drive_pose1, waiting1])

        # ----------------- Pick ---------------------
        s_init_pose20 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        s_init_pose21 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        pose_est20 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj})
        s_move20 = MovePose.MOVEPROOT(name="Top1",
                                      controller_ns=controller_ns,
                                      action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move21 = MovePose.MOVEP(name="Top2", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move22 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                               action_goal=blackboard.gripper_open_pos,
                               force=blackboard.gripper_open_force)        
        s_move23 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_pose"})
        s_move24 = Gripper.GOTO(name="Close", controller_ns=controller_ns,
                               action_goal=blackboard.gripper_close_pos,
                               force=blackboard.gripper_close_force)        
        s_move25 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})

        pick = py_trees.composites.Sequence(name="Pick")
        pick.add_children([pose_est20, s_init_pose20, s_move20, s_move21, s_move22, s_move23, s_move24, s_move25, s_init_pose21])


        # ----------------- Delivery ---------------------
        deliver = py_trees.composites.Sequence(name="Deliver")
        s_drive_pose3 = MoveJoint.MOVEJ(name="DrivePose", controller_ns=controller_ns,
                                  action_goal=blackboard.drive_config)
        pose_est3 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot':robot,'destination': destination})
        ticketing3 = Ticketing(child=pose_est3, idx=idx, name="Ticketing")
        s_drive3 = MoveBase.MOVEB(name="Drive",idx=idx, destination=destination,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"})
        replanning3 = Replanning(s_drive3, idx=idx, name="Replan")
        waiting3 = py_trees.composites.Parallel(name='Waiting', children=[ticketing3, replanning3])
        deliver.add_children([s_drive_pose3, waiting3])

        # ----------------- Place ---------------------
        place = py_trees.composites.Sequence(name="Place")
        s_init_pose40 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        s_init_pose41 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        pose_est4 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj,
                                                             'destination':destination})
        s_move40 = MovePose.MOVEPROOT(name="Top1",
                                      controller_ns=controller_ns,
                                      action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move41 = MovePose.MOVEP(name="Top2", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move42 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_pose"})
        s_move43 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force)        
        s_move44 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        
        s_move45 = WorldModel.REMOVE(name="Remove", target=obj)
        place.add_children([pose_est4, s_init_pose40, s_move40, s_move41, s_move42, s_move43, s_move44, s_move45, s_init_pose41])
        
        task = py_trees.composites.Sequence(name="Delivery")
        task.add_children([bring, pick, deliver, place])
        return task


