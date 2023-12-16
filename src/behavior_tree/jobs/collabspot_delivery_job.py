
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
        # self.blackboard.gripper_open_pos = rospy.get_param("gripper_open_pos")
        # self.blackboard.gripper_close_pos = rospy.get_param("gripper_close_pos")
        # self.blackboard.gripper_open_force = rospy.get_param("gripper_open_force")
        # self.blackboard.gripper_close_force = rospy.get_param("gripper_close_force")
        # self.blackboard.init_config = eval(rospy.get_param("init_config", [0, -np.pi/2., np.pi/2., -np.pi/2., -np.pi/2., np.pi/4.]))
        # self.blackboard.drive_config = [np.pi/2, -2.4, 2.4, -np.pi/2., -np.pi/2., 0]

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
        print("SPOT!!!!!!\n\n\n\n\n\n\n\n", robot, obj, source, destination)
        '''
        "2: (WAITDRIVE picking_station1_collab_spot_parking_target haetae)",
        "3: (COLLABDRIVE placing_shelf1_collab_spot_parking_target)",
        '''

        ## wiatdrive

        # bring = py_trees.composites.Sequence(name="Bring")
        # s_drive_pose1 = MoveJoint.MOVEJ(name="DrivePose", controller_ns=controller_ns,
        #                           action_goal=blackboard.drive_config)
        # pose_est10 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
        #                                       object_dict = {'robot':robot,'destination': source})
        # ticketing1 = Ticketing(child=pose_est10, idx=idx, name="Ticketing")
        # s_drive10 = MoveBase.MOVEB(name="Drive", idx=idx, destination=source,
        #                            action_goal={'pose': "Plan"+idx+"/parking_pose"})
        # replanning1 = Replanning(s_drive10, idx=idx, name="Replan")
        # waiting1 = py_trees.composites.Parallel(name='Waiting', children=[ticketing1, replanning1])
        # bring.add_children([s_drive_pose1, waiting1])
        
        real_source = None
        real_destination = None

        # if source == "picking_station1":
        #     real_source = "picking_station1_collab_spot_parking_target"
        # elif source == "picking_station2":
        #     real_source = "picking_station2_collab_spot_parking_target"
        # else:
        #     raise NotImplementedError()

        # if destination == "placing_shelf1":
        #     real_destination = "placing_shelf1_collab_spot_parking_target"
        # elif destination == "placing_shelf2":
        #     real_destination = "placing_shelf2_collab_spot_parking_target"
        # else:
        #     raise NotImplementedError()

        waitdrive = py_trees.composites.Sequence(name="WaitDrive")
        pose_est1 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot': robot, 'destination': source})
        ticketing1 = Ticketing(child=pose_est1, idx=idx, name="Ticketing")
        s_drive1 = MoveBase.MOVEB(name="Navigate", idx=idx,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"}, destination=source)
        replanning1 = Replanning(s_drive1, idx=idx, name="Replan")
        waiting1 = py_trees.composites.Parallel(name='Waiting', children=[ticketing1, replanning1])

        target = 'haetae'

            ## wait wether "target" arrives "destination" or not?
        sync_pose_est1 = WorldModel.SYNC_POSE_ESTIMATOR_WAIT(name="Sync"+idx, target_obj=target, placement=source)
        wait_condition1 = py_trees.decorators.Condition(name="Wait"+idx, child=sync_pose_est1, status=py_trees.common.Status.SUCCESS)

        waitdrive.add_children([waiting1, wait_condition1])

        ## collabdrive
        collabdrive = py_trees.composites.Sequence(name="CollabDrive")
            ## wait wether "LOADING" is complete
        sync_pose_est2 = WorldModel.SYNC_POSE_ESTIMATOR_LOAD(name="Sync"+idx, target_obj='spot')
        wait_condition2 = py_trees.decorators.Condition(name="WaitLoad"+idx, child=sync_pose_est2, status=py_trees.common.Status.SUCCESS)
        pose_est2 = WorldModel.PARKING_POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'robot': robot, 'destination': destination, 'collab': True})
        s_drive2 = MoveBase.MOVEBCOLLAB(name="Navigate", idx=idx,
                                   action_goal={'pose': "Plan"+idx+"/parking_pose"}, destination=destination, source=source)

        sync_pose_est3 = WorldModel.SYNC_POSE_ESTIMATOR_DELETE(name="Sync"+idx, target_obj=obj)
        wait_condition3 = py_trees.decorators.Condition(name="WaitDelete"+idx, child=sync_pose_est3, status=py_trees.common.Status.SUCCESS)

        collabdrive.add_children([wait_condition2, pose_est2, s_drive2, wait_condition3])
        
        task = py_trees.composites.Sequence(name="CollabDelivery")
        # task.add_children([bring, pick, deliver, place])

        # task.add_children([waitdrive])

        task.add_children([waitdrive, collabdrive])

        return task
