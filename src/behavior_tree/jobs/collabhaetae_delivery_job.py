
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
from behavior_tree.decorators import Ticketing, Replanning, Reconfiguring

from geometry_msgs.msg import Pose, Point, Quaternion
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
        # self._subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
        self._name = "collab_delivery"
        self._goal = None
        self._lock = threading.Lock()
        
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.gripper_open_pos = rospy.get_param("gripper_open_pos")
        self.blackboard.gripper_close_pos = rospy.get_param("gripper_close_pos")
        self.blackboard.gripper_open_force = rospy.get_param("gripper_open_force")
        self.blackboard.gripper_close_force = rospy.get_param("gripper_close_force")
        self.blackboard.init_config = eval(rospy.get_param("init_config", "[0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.]"))
        self.blackboard.drive_config = eval(rospy.get_param("drive_config", "[0, 0, 0, 0., 0., 0]"))

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
            rospy.logerr("MOVE: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range( len(list(grounding.keys())) ):
                if grounding[str(i+1)]['primitive_action'] in ['collab_delivery']:
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
        root = py_trees.composites.Sequence(name="CollabDelivery")
        blackboard = py_trees.blackboard.Blackboard()
        
        if goal[idx]["primitive_action"] in ['collab_delivery']:
            robot       = goal[idx]['robot']
            obj         = goal[idx]['object']
            source      = goal[idx]['source']
            destination = goal[idx]['destination']
        else:
            return None

        plate_position = "spot_table_from_gz"
        # plate_position = "spot_table_for_boxl"

        '''
            "7: (WAITDRIVE picking_station1_collab_haetae_parking_target spot)",
            "4: (COLLABLOAD box_l spot_table_from_gz)",
            "5: (COLLABUNLOAD box_l placing_shelf1_unload_target)",
            "7: (DRIVE picking_station1_collab_haetae_parking_target)",
        '''

        ## waitdrive 
        waitdrive = py_trees.composites.Sequence(name="WaitDrive")
        s_drive_pose1 = MoveJoint.MOVEJ(name="DrivePose", controller_ns=controller_ns,
                                  action_goal=blackboard.drive_config)

        pose_est1 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot': robot, 'destination': source, 'collab_side': True})

        # ticketing1 = Ticketing(child=pose_est1, idx=idx, name="Ticketing")
        s_drive1 = MoveBase.MOVEB(name="Navigate", idx=idx,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"}, destination=source)
        # replanning1 = Replanning(s_drive1, idx=idx, name="Replan")
        # waiting1 = py_trees.composites.Parallel(name='Waiting', children=[ticketing1, replanning1])
        approaching1 = MoveBase.TOUCHB(name="Touch", idx=idx,
                                      action_goal={'pose': "Plan"+idx+"/parking_pose"}, destination=source)
        
        target = 'spot'

            ## wait wether "target" arrives "destination" or not?
        sync_pose_est1 = WorldModel.SYNC_POSE_ESTIMATOR_WAIT(name="Sync"+idx, target_obj=target, placement=source)
        wait_condition1 = py_trees.decorators.Condition(name="Wait"+idx, child=sync_pose_est1, status=py_trees.common.Status.SUCCESS)

        # waitdrive.add_children([pose_est1, waiting1, wait_condition1])
        waitdrive.add_children([s_drive_pose1, pose_est1, s_drive1, approaching1, wait_condition1])

        ## collabload
        s_init2_1 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)

        pose_est2_1 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj})
        s_move2_10 = MovePose.MOVEPROOT(name="Top1", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})
        s_move2_11 = MovePose.MOVEP(name="Top2", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_top_pose"})     
        s_move2_13 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/grasp_pose"})
        s_move2_14 = Gripper.GOTO(name="Close", controller_ns=controller_ns,
                                action_goal=blackboard.gripper_close_pos,
                                force=blackboard.gripper_close_force)        
        pose_est2_2 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target': obj,
                                                             'destination': plate_position})
        s_move2_22 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_pose"})
        pick = py_trees.composites.Sequence(name="CollabLoad")
        pick.add_children([pose_est2_1, s_init2_1, s_move2_10, s_move2_11, s_move2_13, s_move2_14, pose_est2_2, s_move2_22])


        place = py_trees.composites.Sequence(name="CollabUNLoad")
        if source == "picking_station1":
            if destination == "placing_shelf1":
                unload_destination = "ps1_unload_target"
            elif destination == "placing_shelf2":
                unload_destination = "ps2_unload_target"
            else:
                raise NotImplementedError()
        elif source == "picking_station2":
            if destination == "placing_shelf1":
                unload_destination = "ps1_unload_target2"
            elif destination == "placing_shelf2":
                unload_destination = "ps2_unload_target2"
            else:
                raise NotImplementedError()

        ## collabunload

        sync_pose_est3 = WorldModel.SYNC_POSE_ESTIMATOR_HAETAE2(name="Sync"+idx, destination=destination) ## spot이 destination에 도착할 때 까지.
 
        wait_condition3 = py_trees.decorators.Condition(name="Wait"+idx, child=sync_pose_est3, status=py_trees.common.Status.SUCCESS)
        
        pose_est3_1 = WorldModel.POSE_ESTIMATOR(name="Plan222"+idx,
                                              object_dict = {'target': obj,
                                                             'destination': unload_destination})
        s_move3_20 = MovePose.MOVEPROOT(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan222"+idx+"/place_top_pose"})
        s_move3_21 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan222"+idx+"/place_top_pose"})
        s_move3_22 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan222"+idx+"/place_pose"})
        
        spot_leave = Communicate.SpotLeave(name="SpotGO", unload='spot')
        s_move3_23 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                               action_goal=blackboard.gripper_open_pos,
                               force=blackboard.gripper_open_force)        
        
        s_move3_24 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan222"+idx+"/place_top_pose"})
        s_init_pose41 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns,
                                  action_goal=blackboard.init_config)
        # s_init_pose42 = MoveJoint.MOVEJ(name="DrivePose", controller_ns=controller_ns,
        #                           action_goal=blackboard.drive_config)
        ############################

        s_move3_25 = WorldModel.REMOVE(name="Remove", target=obj)

        # place.add_children([wait_condition3, pose_est3_1, s_move3_20, s_move3_21, s_move3_22,s_move3_23,s_move3_24, s_move3_25, s_init_pose41, s_init_pose42])
        place.add_children([wait_condition3, pose_est3_1, s_move3_20, s_move3_21, s_move3_22, spot_leave, s_move3_23,s_move3_24, s_move3_25, s_init_pose41])
        task = py_trees.composites.Sequence(name="CollabDelivery")
        task.add_children([ waitdrive, pick, place])

        return task
