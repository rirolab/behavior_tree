
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
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose

sys.path.insert(0,'..')
from subtrees import MoveJoint, MovePose, Gripper, Stop, WorldModel


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
            for i in range( len(grounding.keys()) ):
                if grounding[str(i+1)]['primitive_action'] in ['handover']:
                    self.goal = grounding #[str(i+1)] )
                    break
                
            ## if len(self.blackboard.groundings)>0 and \
            ##   self.blackboard.grounding[0]['primitive_action'].find('move')>=0:              
            ##   self.goal = msg

            
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
        root = py_trees.composites.Sequence(name="Handover")
        blackboard = py_trees.blackboard.Blackboard()

        if goal[idx]["primitive_action"] in ['handover']:
            obj = goal[idx]['object']
            destination = goal[idx]['destination']
        else:
            return None

        # TODO
        if destination=='na':
            destination = None
        
        # ----------------- Init Task ----------------        
        s_init = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)

        # ----------------- Handover ---------------------
        s_move1 = MoveJoint.MOVEJ(name="Front", controller_ns=controller_ns,
                                  action_goal=[np.pi/2, -np.pi/2., np.pi/2., np.pi/2., -np.pi/2., 0])
        s_move2 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                                   action_goal=blackboard.gripper_open_pos, check_contact=True)        

        wm_remove = WorldModel.REMOVE(name="Delete", action_goal={'obj_name': obj})


        task = py_trees.composites.Sequence(name="Handover")
        task.add_children([s_move1, s_move2, s_init, wm_remove])
        return task


    
        ## run_or_cancel = py_trees.composites.Selector(name="Run or Cancel?")

        ## cancel_seq = py_trees.composites.Sequence(name="Cancel")        
        ## is_stop_requested = py_trees.blackboard.CheckBlackboardVariable(
        ##     name="Stop?",
        ##     variable_name="stop_cmd",
        ##     expected_value=True
        ##     )
        ## cancel_seq.add_child(is_stop_requested)                                
        
        ## run_or_cancel.add_children([cancel_seq, pick])
        ## root.add_child(run_or_cancel)
        ## return root
