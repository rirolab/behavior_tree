import os
import numpy as np
import subprocess
import signal

import py_trees
import rospy
import rospy, rospkg

import std_msgs.msg as std_msgs
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryResult

from os import listdir


def get_filenumber(data_path, prefix="temp"):
    
    files = [f for f in listdir(data_path) \
             if os.path.isfile(os.path.join(data_path, f))]

    done = False
    for f in files:
        if f.find(prefix)>=0:
            try:
                num = int(f.split('.')[0].split('_')[-1])
            except:
                return 0
            done = True
            
    if done is False:
        return 0
    return num+1


class ROSBAG(py_trees.behaviour.Behaviour):
    """
    Note that this behaviour will return with
    :attr:`~py_trees.common.Status.SUCCESS`. It will also send a clearing
    command to the robot if it is cancelled or interrupted by a higher
    priority behaviour.
    """

    def __init__(self, name, action_goal=None,
                 filename=None, controller_ns="", topic_list=None):
        super(ROSBAG, self).__init__(name=name)

        self.action_goal   = action_goal
        self.controller_ns = controller_ns
        self.topic_list    = topic_list
        self.sent_goal     = False
        self.collector     = None

        if filename is not None:
            self.filename = filename
        else:
            rospack   = rospkg.RosPack()
            data_path = os.path.join(rospack.get_path('dialogue_irl'), \
                                         "data/rosbag")
            num       = get_filenumber(data_path, prefix="jog")
            self.filename = os.path.join(data_path, "jog_"+str(num) )
                                         
    def update(self):
        self.logger.debug("%s.update()" % self.__class__.__name__)

        ## if self.collector is None:
        ##     self.feedback_message = \
        ##       "no collection client, did you call setup() on your tree?"
        ##     return py_trees.Status.FAILURE

        if not self.sent_goal:
            cmd_str = '/opt/ros/kinetic/lib/rosbag/record -o {} {}'.format(self.filename,
                                                             self.topic_list)
            self.collector = subprocess.Popen(cmd_str, shell=True, stdin=subprocess.PIPE)
            
            self.sent_goal = True
            self.feedback_message = "Launching a recording script"
            return py_trees.common.Status.RUNNING

        ## state = self.arm.get_state()
        ## ret   = self.arm.get_result()
        
        ## if  state in [GoalStatus.ABORTED,
        ##               GoalStatus.PREEMPTED] and \
        ##               ret != FollowJointTrajectoryResult.SUCCESSFUL: 
        ##     self.feedback_message = "FAILURE"
        ##     return py_trees.common.Status.FAILURE

        ## if ret == FollowJointTrajectoryResult.SUCCESSFUL:
        ##     self.feedback_message = "SUCCESSFUL"
        ##     return py_trees.common.Status.SUCCESS
        ## else:
        ##     self.feedback_message = "RUNNING"
        return py_trees.common.Status.RUNNING
            

        ## ret = self.arm.moveJoint([-1.57,-1.57,0,-1.57,0,0], timeout=3.0)
        ## if ret<0: return py_trees.common.Status.FAILURE
        ## elif ret==0: return py_trees.common.Status.RUNNING
        
        ## ret = self.arm.moveJoint([-1.57,-0.1745,-2.79,-1.57,-3.14,0], timeout=3.0)
        ## if ret==2 or ret==3:
        ##     return py_trees.common.Status.FAILURE
        ## elif ret==1:
        ## elif ret==0:
    

    def terminate(self, new_status):
        print self.collector
        if self.collector is not None:
            terminate_process_and_children(self.collector)
        return

def terminate_process_and_children(p):
    import psutil
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
        p.wait()  # we wait for children to terminate
    #p.terminate()
