
import copy, sys
import move_base_msgs.msg as move_base_msgs
import py_trees, py_trees_ros
import rospy
import threading
import numpy as np
import json
import PyKDL

import std_msgs.msg as std_msgs
from geometry_msgs.msg import Pose, Quaternion #PointStamped,
from ur5_srvs.srv import String_Pose, String_PoseResponse
from complex_action_client import misc

sys.path.insert(0,'..')
from subtrees import MovePose, Gripper, Rosbag


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
        
        self._subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
        self._goal = None
        self._lock = threading.Lock()
        ## self.blackboard = py_trees.blackboard.Blackboard()

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
                if grounding[str(i+1)]['primitive_action'].find('jog')>=0:
                    self.goal = grounding #[str(i+1)]
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
    def create_root(idx="1", goal=std_msgs.Empty(), controller_ns="", rec_topic_list=None):
        """
        Create the job subtree based on the incoming goal specification.

        Args:
            goal (:class:`~std_msgs.msg.Empty`): incoming goal specification

        Returns:
           :class:`~py_trees.behaviour.Behaviour`: subtree root
        """
        if not ( goal[idx]["primitive_action"] in ['jog'] ):
            return None
        
        # beahviors
        ## root = py_trees.composites.Sequence(name="Jog")
        root = py_trees.composites.Parallel(name="Jog",\
                                            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # ----------------------------- Custom ----------------------------
        frame     = "l_palm" #goal['frame'].encode('ascii','ignore')
        direction = goal['direction'].encode('ascii','ignore')
        dist      = goal['distance']

        # Enable a continous jog motion
        if dist == 'na':
            # This offset value should be higher than pos or angle deadzon defined in arm client.
            if direction.find('rx')>=0 or direction.find('ry')>=0 or \
              direction.find('rz')>=0: 
                dist = np.pi/180.0*1.
            else:
                dist = 0.01
            cont = True
        else:
            cont = False
            
        if direction.find('-')>=0: dist *= -1.

        pose = Pose()        
        if direction.find('rx')>=0:
            q = PyKDL.Rotation.RotX(dist).GetQuaternion()
            pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        elif direction.find('ry')>=0:
            q = PyKDL.Rotation.RotY(dist).GetQuaternion()
            pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        elif direction.find('rz')>=0:
            q = PyKDL.Rotation.RotZ(dist).GetQuaternion()
            pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        elif direction.find('x')>=0:
            pose.position.x = dist
        elif direction.find('y')>=0:
            pose.position.y = dist
        elif direction.find('z')>=0:
            pose.position.z = dist

        goal_dict = {'pose': pose,
                     'frame': frame}
        
        s_move  = MovePose.MOVEPR(name="jogmotion", controller_ns=controller_ns,
                                  action_goal=goal_dict, cont=cont)
        # --------------------------------------------------------------------

        run_or_cancel = py_trees.composites.Selector(name="Run or Cancel?")

        cancel_seq = py_trees.composites.Sequence(name="Cancel")        
        is_stop_requested = py_trees.blackboard.CheckBlackboardVariable(
            name="Stop?",
            variable_name="stop_cmd",
            expected_value=True
            )
        ## s_stop = Stop.STOP(name="Stop", controller_ns=controller_ns,
        ##                    action_goal=50)
        cancel_seq.add_child(is_stop_requested)#, s_stop])        
        run_or_cancel.add_children([is_stop_requested, s_move])

        if rec_topic_list is None or len(rec_topic_list)==0:
            root.add_child(run_or_cancel)
        else:
            logger = Rosbag.ROSBAG(name="logger", topic_list=rec_topic_list)
            root.add_children([run_or_cancel, logger])

        return root
