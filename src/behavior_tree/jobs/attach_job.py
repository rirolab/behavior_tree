import copy, sys
import rospy 
import py_trees, py_trees_ros
import threading 
import tf
import numpy as np 
import PyKDL
import std_msgs.msg as std_msgs 
from subtrees import MoveJoint, MovePose, Gripper, Stop, WorldModel
import json

class Move(object):

    def __init__(self):

        self._grounding_channel = "symbol grounding"

        self.subscriber = rospy.Subscriber(self._grounding_channel, std_msgs.String, self.incoming)
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

        with self._lock:
            g = copy.copy(self._goal) or self._goal
        return g 

    @goal.setter
    def goal(self, value):

        with self._lock:
            self._goal = value 
    
    def incoming(self, msg):

        if self.goal:
            rospy.logerr("MOVE: rejecting new goal, previous still in the pipeline")
        else:
            grounding = json.loads(msg.data)['params']
            for i in range (len(grounding.keys())):
                if grounding[str(i+1)]['primitive_action']
                    self.goal = grounding
                break

    @staticmethod 
    def create_root(idx="1", goal=std_msgs.Empty(), controller_ns="", **kwargs):
        
        root = py_trees.composites.Sequence(name="Attach")
        blackboard = py_trees.blackboard.Blackboard()

        if goal[idx]["primitive_action"] in ['move']:
            obj         = goal[idx]['object'].encode('ascii','ignore')
            destination = goal[idx]['destination'].encode('ascii','ignore')
        else:
            return None

       
        if destination=='na':
            destination='plate'
            print("no plate, so selected plate as a destination")


        #s_init3 = MoveJoint.MOVEJ(name="Init", controller_ns=controller_ns, 
                                   # action_goal=blackboard.init_config)

        pose_est2 = WorldModel.POSE_ESTIMATOR(name="Plan"+idx,
                                              object_dict = {'target':obj,
                                                             'destination': destination})

        s_move21 = MovePose.MOVEPROOT(name="Top1",
                                      controller_ns=controller_ns,
                                      action_goal={'pose': "Plan"+idx+"/place_top_pose"})
        s_move22 = MovePose.MOVEP(name="Approach", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_pose"})
        s_move23 = Gripper.GOTO(name="Open", controller_ns=controller_ns,
                                action_goal=blackboard.gripper_open_pos,
                                force=blackboard.gripper_open_force)        
        s_move24 = MovePose.MOVEP(name="Top", controller_ns=controller_ns,
                                 action_goal={'pose': "Plan"+idx+"/place_top_pose"})

        attach = py_trees.composites.Sequence(name="Attach")
        attach.add_children([pose_est2, s_move21, s_move22, s_move23, s_move24])

        return attach

    

        

    
