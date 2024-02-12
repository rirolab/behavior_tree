#!/usr/bin/env python3
# Copyright YYYY Massachusetts Institute of Technology 

import importlib
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import std_msgs.msg as std_msgs
import sys
import numpy as np
import json
from std_srvs.srv import Empty, EmptyResponse

## import subtrees.WM2Blackboard
from subtrees import Grnd2Blackboard, WM2Blackboard, TS2Blackboard, Idle
import decorators

def create_root(controller_ns="", robot_type=''):
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.
    
    Returns:
    :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = py_trees.composites.Parallel("ROOT")

    # ---------------- Root->Blackboard ------------------------
    ## topics2bb = py_trees.composites.Sequence("Topics2BB")
    grnd2bb = Grnd2Blackboard.ToBlackboard(name="Grnd2BB",
                                           topic_name="symbol_grounding")
    wm2bb   = WM2Blackboard.ToBlackboard(name="WM2BB",
                                         topic_name="/world_model" )
    ts2bb   = TS2Blackboard.ToBlackboard(name="TS2BB",
                                         topic_name="/task_status")
    ## reccmd2bb = py_trees_ros.subscribers.EventToBlackboard(
    ##     name="RecCmd2BB",
    ##     topic_name="/record_cmd",
    ##     variable_name="record_cmd"
    ##     )
    #root.add_children([topics2bb, reccmd2bb])
    ## topics2bb.add_child(grnd2bb)

    # ---------------- Root->Log ------------------------
    ## topics2log = py_trees.composites.Sequence("Topics2Log")
    ## is_move = py_trees.blackboard.CheckBlackboardVariable(
    ##     name="Record?",
    ##     variable_name="record_cmd",
    ##     expected_value=True
    ##     )
    ## if len(rec_jobs)>0:
    ##     for job in rec_jobs:
    ##         module_name = '.'.join(job.split('.')[:-1])
    ##         class_name = job.split('.')[-1]
    ##         self.jobs.append(getattr(importlib.import_module(module_name), class_name)())

    ##                 job_root = job.create_root(job.goal, self.controller_ns)
    ##                 if not job_root.setup(timeout=15):
    ##                     rospy.logerr("{0}: failed to setup".format(job.name))
    ##                     continue
    ##                 tree.insert_subtree(job_root, self.priorities.id, 0)            
    ## topics2log.add_child()

    # ---------------- Root->Priorities- -----------------------
    priorities = py_trees.composites.Selector("Priorities")
    idle = Idle.Idle(name="Idle", type=robot_type, controller_ns=controller_ns)
    # idle       = py_trees.behaviours.Running(name="Idle")
    priorities.add_child(idle)
    
    # root.add_children([grnd2bb,priorities])
    root.add_children([grnd2bb, wm2bb ,ts2bb, priorities])

    return root


def load_topic_list(filename):
    if filename is None: return None
        
    topic_list_str = ""
    with open(filename) as json_file:
        data = json.load(json_file)
        for p in data['topic_list']:
            topic_list_str += p+' '
    if topic_list_str == "": return None
    return topic_list_str
            

class SplinteredReality(object):

    def __init__(self, jobs, robot_name, robot_type, rec_topic_list=None, n_loop=1,
                 enable_inf_loop=False, loop_timeout=-1):
        """
        Initialise a core tree (minus a job) and preload job classes ready to
         be used in spinning up and running the job later when requested.

        Args:
            jobs ([:obj:`str`]): list of module names as strings (e.g. 'py_trees_ros.tutorials.jobs.Scan')
        """
        self.controller_ns   = rospy.get_param("controller_ns", "")
        self.rec_topic_list  = rec_topic_list
        
        self.n_loop          = n_loop
        self.enable_inf_loop = enable_inf_loop
        self.loop_timeout    = loop_timeout 
        
        self.blackboard            = py_trees.blackboard.Blackboard()
        self.blackboard.is_running = False
        self.blackboard.robot_name = robot_name
        
        self.tree = py_trees_ros.trees.BehaviourTree(create_root(self.controller_ns, robot_type=robot_type))
        self.tree.add_pre_tick_handler(self.pre_tick_handler)
        self.tree.add_post_tick_handler(self.post_tick_handler)
        self.report_publisher = rospy.Publisher("~report", std_msgs.String, queue_size=5)
        self.jobs = []
        for job in jobs:
            module_name = '.'.join(job.split('.')[:-1])
            class_name = job.split('.')[-1]
            self.jobs.append(getattr(importlib.import_module(module_name), class_name)())
        self.current_job = None
        rospy.loginfo("dynamic_behavior_tree: initialized")


    def setup(self):
        """
        Redirect the setup function"

        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        rospy.loginfo('root node setup start')
        return self.tree.setup(timeout=15)


    def pre_tick_handler(self, tree):
        """
        Check if a job is running. If not, spin up a new job subtree
        if a request has come in.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
        rospy.loginfo(f"[TREE ROOT] pre_tick_handler() called jobs: {self.blackboard.get('grnd_msg')}!!!")
        goal = self.blackboard.get('grnd_msg')
        
        if goal is not None:
            goal = json.loads(goal.data)['params']
            task_list = []

            if self.idle():
                cancel_seq = py_trees.composites.Sequence(name="Cancel")        
                is_stop_requested = py_trees.blackboard.CheckBlackboardVariable(
                    name="Stop?",
                    variable_name="stop_cmd",
                    expected_value=True
                    )
                cancel_seq.add_child(is_stop_requested)                                

                task = py_trees.composites.Sequence(name="Task")
                        
                if self.n_loop<=1 and self.enable_inf_loop is False:
                    run_or_cancel = py_trees.composites.Selector(name="Run or Cancel?")
                    run_or_cancel.add_children([cancel_seq, task])
                else:
                    loop = decorators.Loop(child=task, name='Loop', n_loop=self.n_loop,
                                        enable_inf_loop=self.enable_inf_loop,
                                        timeout=self.loop_timeout)

                    run_or_cancel = py_trees.composites.Selector(name="Run or Cancel?")
                    run_or_cancel.add_children([cancel_seq, loop])
                
                ## root = py_trees.composites.Sequence(name="SubRoot")
                ## root.add_child(run_or_cancel)
                root = run_or_cancel
                tree.insert_subtree(root, self.priorities.id, 0)
                rospy.loginfo("{0}: inserted job subtree".format(root.name))

            
            # Dynamic Reconfiguration
            else:
                task = self.priorities.children[0].children[-1] # Task Sequence Node
                        # self.jobs are holding all available job classes
            
            for idx in range(len(goal)):        
                for job in self.jobs:   ##ex) "1: (GRIPPER_CLOSE)"         
                    # job.goal contains current goal json message.
                    # from IPython import embed; embed(); sys.exit()
                    if job.name == goal[str(idx+1)]["primitive_action"]:
                        job_root = job.create_root(str(idx+1), goal,
                                                self.controller_ns,
                                                rec_topic_list=self.rec_topic_list)
                        if job_root is None:
                            continue
                        rospy.loginfo("{0}: running to set up all subtree modules".format(idx+1))
                        if not job_root.setup(timeout=15):
                            rospy.logerr("{0}: failed to setup".format(idx+1))
                            continue
                        rospy.loginfo("{0}: finished setting up".format(idx+1))
                        task_list.append(job_root)
                        break
            task.add_children(task_list)
            self.current_job = task.children[0].name
        return


    def post_tick_handler(self, tree):
        """
        Check if a job is running and if it has finished. If so, prune the job subtree from the tree.
        Additionally, make a status report upon introspection of the tree.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
        # delete the job subtree if it is finished
        if not self.idle():
            job = self.priorities.children[0]
                        
            if job.status == py_trees.common.Status.SUCCESS or job.status == py_trees.common.Status.FAILURE or job.status == py_trees.common.Status.INVALID:
                rospy.loginfo("{0}: finished [{1}]".format(job.name, job.status))
                print("{0}: finished [{1}]".format(job.name, job.status))
                
                tree.prune_subtree(job.id)
                self.current_job = None

    def idle(self):
        return len(self.priorities.children) == 1

    def busy(self):
        """
        Check if a job subtree exists and is running.

        Returns:
            :obj:`bool`: whether it is busy with a job subtree or not
        """
        return len(self.priorities.children) == 3

    @property
    def priorities(self):
        return self.tree.root.children[-1]

    def tick_tock(self):
        self.tree.tick_tock(500)

    def shutdown(self):
        self.tree.interrupt()

    def _bt_stop_srv(self, req):
        print(f"==================\nservice requested!! value = {self.blackboard.get('stop_cmd')}\n==================")
        self.blackboard.set("stop_cmd", True)
        print(f"==================\nservice requested!! value = {self.blackboard.get('stop_cmd')}\n==================")
        return EmptyResponse()
    
    def _bt_restart_srv(self, req):
        print(f"==================\nservice requested!! value = {self.blackboard.get('stop_cmd')}\n==================")
        self.blackboard.set("stop_cmd", False)
        return EmptyResponse()
    


##############################################################################
# Main
##############################################################################

if __name__ == '__main__':
    """
    Entry point for the demo script.
    """
    import optparse
    p = optparse.OptionParser()
    p.add_option('--viz', '-v', action='store_true', dest='viz',
                 default=False,
                 help='use visualization code for rviz')
    p.add_option('--rec_topics', '-t', action='store', dest='topic_json',
                 default=None, help='a list of topic to record')
    p.add_option('--robot', '-r', action='store', dest='robot',help='type of robot. choose manip or quad')
    p.add_option('--robot_name', '-n', action='store', dest='robot_name',help='name of robot. haetae, spot, stretch')
    
    opt, args = p.parse_args()

    # load the list of topics for recording
    if opt.topic_json is None or opt.topic_json.find('None')>=0:
        topic_list = None
    else:
        topic_list = load_topic_list(opt.topic_json)

    #py_trees.logging.level = py_trees.logging.Level.DEBUG

    # TODO: import a list of jobs from a json file or ROS parameter server.
    # Keep the default job on the top
    rospy.init_node("tree")
    rospy.loginfo("Robot Type: {0}".format(opt.robot))
    jobs = []
    if opt.robot == 'manip':
        jobs = [
                'jobs.delivery_job.Move',
                'jobs.home_job.Move',
                'jobs.drive_job.Move',
                'jobs.collabhaetae_delivery_job.Move',
                'jobs.help_load_job.Move',
                'jobs.help_unload_job.Move',
                ]
    elif opt.robot == 'quad':
        jobs = ['jobs.drive_job.Move',
                'jobs.spot_delivery_job.Move',
                'jobs.home_job.Move',
                'jobs.collabspot_delivery_job.Move'
                ]
        
    else:
        print(f"Given robot type should be one of 'manip' and 'quad'. Input type: {opt.robot} ")
        raise NotImplementedError()
    rospy.loginfo(jobs)
    if opt.robot_name not in ['haetae', 'spot', 'stretch']:
        print(f"Given robot type should be one of spot/haetae/stretch. Input type: {opt.robot_name} ")
        raise NotImplementedError

    splintered_reality = SplinteredReality(jobs=jobs, rec_topic_list=topic_list, robot_name=opt.robot_name, robot_type=opt.robot)
    rospy.on_shutdown(splintered_reality.shutdown)
    if not splintered_reality.setup():
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    splintered_reality.tick_tock()

    
