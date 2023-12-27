
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
        self._name = ""
        self._goal = None
        self._lock = threading.Lock()
        
        self.blackboard = py_trees.blackboard.Blackboard()
        
        ## self.object      = None
        ## self.destination = None

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
            rospy.logerr("(collabdrive) MOVE: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range( len(list(grounding.keys())) ):
                if grounding[str(i+1)]['primitive_action'] in ['collabdrive']:
                    self.goal = grounding #[str(i+1)] )
                    break


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
        root = py_trees.composites.Sequence(name="CollabDrive")
        blackboard = py_trees.blackboard.Blackboard()
        
        if goal[idx]["primitive_action"] in ['collabdrive']:
            destination = goal[idx]['destination']
        else:
            return None

        # ----------------- Bring ---------------------
        pose_est10 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'destination': destination, 'collab': True})
        s_drive10 = MoveBase.MOVEBCOLLAB(name="Navigate", 
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"}, is_collab=True , destination=destination)
        # s_drive11 = MoveBase.ALIGNB(name="Align", 
        #                            action_goal={'pose': "Plan"+idx+"/near_parking_pose"})
        
        # s_drive12 = MoveBase.TOUCHB(name="Approach")
        # root.add_children([s_drive_pose, pose_est10, s_drive10, s_drive11, s_drive12])

        # sync_pose_est = WorldModel.SYNC_POSE_ESTIMATOR_SPOT(name="Sync"+idx, object_dict={'target1': 'spot', 'target2': 'haetae'}, distance_criteria=1.0)
        sync_pose_est = WorldModel.SYNC_POSE_ESTIMATOR_LOAD(name="Sync"+idx, target_obj='spot')
        wait_condition = py_trees.decorators.Condition(name="WaitLoad"+idx, child=sync_pose_est, status=py_trees.common.Status.SUCCESS)

        # sync_pose_est2 = WorldModel.SYNC_POSE_ESTIMATOR(name="Sync"+idx, object_dict={'target1': 'spot', 'target2': 'haetae'}, distance_criteria=1.0, smaller_than_criteria=False)
        # wait_condition2 = py_trees.decorators.Condition(name="Wait"+idx, child=sync_pose_est2, status=py_trees.common.Status.SUCCESS)

        root.add_children([wait_condition, pose_est10, s_drive10])
        # root.add_children([pose_est10, wait_condition, s_drive10])


        # task = py_trees.composites.Sequence(name="Delivery")
        return root


