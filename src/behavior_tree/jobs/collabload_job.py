
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
## from complex_action_client import misc

sys.path.insert(0,'..')
from subtrees import MoveJoint, MovePose, Gripper, Stop, WorldModel

# import world_model_node as wm

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
        # self.blackboard.init_config = eval(rospy.get_param("init_config", "[0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.]"))
        self.blackboard.init_config = eval(rospy.get_param("init_config", "[0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.]"))
        
        # self.blackboard.init_config = 
        # print("!!!!!!!!!!!!!!!!!!!\n\n\n", self.blackboard.gripper_open_pos)
        
        # exit()
        # self._br = tf.TransformBroadcaster()
        


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
                if grounding[str(i+1)]['primitive_action'] in ['collabload']:
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

        # listener = tf.TransformListener()
        # while not rospy.is_shutdown():
        #     try:
        #         (pos, quat) = listener.lookupTransform("spot/base_link_plate", "box_s_grip_1", rospy.Time(0))
        #         print("!!!!!!!!!!\n\n\n\n\n\n", pos)
        #         assert len(pos) == 3
        #         box_plate_dist = pos[0] ** 2 + pos[1] ** 2 + pso[2] ** 2
        #         if box_plate_dist < 1.0:
        #             break
        #     except:
        #         print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!23232\n\n\n\n\n\n\n\n\n\n")
        #         pass
        
        # raise NotImplementedError


        # beahviors
        root = py_trees.composites.Sequence(name="Collabload")
        blackboard = py_trees.blackboard.Blackboard()

        if goal[idx]["primitive_action"] in ['collabload']:
            if 'object' in goal[idx].keys():
                obj = goal[idx]['object']
            elif 'obj' in goal[idx].keys():
                obj = goal[idx]['obj']
            else:
                rospy.logerr("CollabLoad: No load object")
                sys.exit()        
            destination = goal[idx]['destination']

            if 'destination_offset' in goal[idx].keys():
                print("??????????????????????111")
                raise NotImplementedError
                destination_offset = goal[idx]['destination_offset'] 
            else:
                # destination_offset = [0,0,-0.1,0,0,0]
                destination_offset = [0,0,0.1,0,0,0]

        else:
            return None
        
        # ------------ Compute -------------------------
        s_init1 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        s_init2 = MoveJoint.MOVEJ(name="Init2", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)

        # ----------------- Pick ---------------------
        pose_est1 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj})
        s_move10 = MovePose.MOVEPROOT(name="Top1", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move11 = MovePose.MOVEP(name="Top2", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move12 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force)        
        s_move13 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_pose"})
        s_move14 = Gripper.GOTO(name="Close", controller_ns=controller_ns,
                                action_goal=blackboard.gripper_close_pos,
                                force=blackboard.gripper_close_force)        
        s_move15 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})

        ############################
        pose_est2 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj,
                                                             'destination': destination,
                                                             'destination_offset': destination_offset})

        sync_pose_est = WorldModel.SYNC_POSE_ESTIMATOR(name="Sync"+idx, object_dict={'target1': 'spot', 'target2': 'haetae'}, distance_criteria=1.0, wait_spot_drive=True)
        # print("!!!@!@!#@!#!#!@$!@$#!@$#!@$#!@$!@\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n", blackboard.get("Place4/place_top_pose"), idx, blackboard.get("Plan"+idx+"/place_top_pose"))
        # raise NotImplementedError()

        wait_condition = py_trees.decorators.Condition(name="Wait"+idx, child=sync_pose_est, status=py_trees.common.Status.SUCCESS)

        #from IPython import embed; embed(); sys.exit()
        s_move20 = MovePose.MOVEPROOT(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move21 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move22 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_pose"})
        s_move23 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                               action_goal=blackboard.gripper_open_pos,
                               force=blackboard.gripper_open_force)        
        s_move24 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        ############################


        pick = py_trees.composites.Sequence(name="CollabLoad")
        # pick.add_children([pose_est1, s_init1, s_move10, s_move11, s_move12, s_move13, s_move14, s_move15, s_init2])
        # pick.add_children([pose_est1, s_init1, s_move10, s_move11, s_move13, s_move14, s_move15])

        # pick.add_children([pose_est1, s_init1, s_move10, s_move11, s_move13, s_move14, s_move15, pose_est2, s_move20, s_move21, s_move22, s_move23, s_move24, s_init2])
        # pick.add_children([pose_est1, s_init1, s_move10, s_move11, s_move13, s_move14, s_move15, pose_est2, s_move20, s_move21, s_move22])

        pick.add_children([pose_est1, s_init1, s_move10, s_move11, s_move13, s_move14, pose_est2, s_move22])

        return pick

    
