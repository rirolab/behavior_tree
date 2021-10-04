#!/usr/bin/env python
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

## import subtrees.WM2Blackboard
from subtrees import Grnd2Blackboard
import decorators

def create_root(controller_ns=""):
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
    ## wm2bb   = WM2Blackboard.ToBlackboard(name="WM2BB",
    ##                                      topic_name="/world_model" )
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
    idle       = py_trees.behaviours.Running(name="Idle")
    priorities.add_child(idle)
    
    root.add_children([grnd2bb,priorities])
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

    def __init__(self, jobs, rec_topic_list=None, n_loop=1,
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
        
        self.tree = py_trees_ros.trees.BehaviourTree(create_root(self.controller_ns))
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
        return self.tree.setup(timeout=15)


    def pre_tick_handler(self, tree):
        """
        Check if a job is running. If not, spin up a new job subtree
        if a request has come in.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """

        goal = None
        for job in self.jobs:
            if job.goal is not None:
                goal = job.goal

        if not self.busy() and goal is not None:
            cancel_seq = py_trees.composites.Sequence(name="Cancel")        
            is_stop_requested = py_trees.blackboard.CheckBlackboardVariable(
                name="Stop?",
                variable_name="stop_cmd",
                expected_value=True
                )
            cancel_seq.add_child(is_stop_requested)                                
            task_list = []

            # self.jobs are holding all available job classes
            for idx in range(len(goal)):                
                for job in self.jobs:
                    # job.goal contains current goal json message.
                    if job.goal is not None:
                        job_root = job.create_root(str(idx+1), job.goal,
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
                        #continue
                        break

            task = py_trees.composites.Sequence(name="Task")
            task.add_children(task_list)
                    
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

            # Reset goals
            for job in self.jobs:
                job.goal = None
            self.current_job = job
            
            return
            ## self.blackboard.is_running = True

            ## if rec_job is not None:
            ##     # launch rosbag?
            ##     rospy.loginfo("Start to record rosbag")

        # Dynamic Reconfiguration
        elif self.busy() and goal is not None and False:
            # Todo find the node by name
            is_task_node = False
            for task_node in self.priorities.iterate():
                if task_node.name == 'Task':
                    is_task_node = True
                    break
            if is_task_node is False:
                rospy.logerr("No Task Node")
            
            ## task = self.priorities.children[0].children[-1].children[-1]
            ## if task.name is not 'Task':
            ##     rospy.logerr("Tree name does not match")
            ## from IPython import embed; embed(); sys.exit()
            
            # Check from the last goal
            for idx in range(1,len(goal)+1):
                if "action" not in goal[str(len(goal)-idx+1)].keys() and \
                  "primitive_action" in goal[str(len(goal)-idx+1)].keys():
                    goal[str(len(goal)-idx+1)]["action"] = goal[str(len(goal)-idx+1)]["primitive_action"]
                
                print "Check: {}th plan".format(str(len(goal)-idx)), \
                  goal[str(len(goal)-idx+1)]["action"]
                
                if len(task_node.children) >= idx and \
                  task_node.children[-idx].name == goal[str(len(goal)-idx+1)]["action"]:
                    rospy.loginfo("{0}: passing to set up".format(len(goal)-idx+1))
                    continue

                # check if there are removable plans
                is_rm_plan = False                
                for i in range(idx+1,len(task_node.children)+1):
                    if task_node.children[-i].name == goal[str(len(goal)-idx+1)]["action"]:
                        is_rm_plan = True
                        break

                if is_rm_plan:
                    blackboard = py_trees.Blackboard()
                    edges      = blackboard.get('edges')
                    if edges is None: edges = []
                        
                    for j in range(idx,i)[::-1]:
                        # remove the name from the known edge set
                        cur_edge = eval(task_node.children[-j].name)
                        edges.remove(cur_edge)
                        task_node.remove_child_by_id(task_node.children[-j].id)
                    blackboard.edges = edges
                else:
                    for job in self.jobs:
                        # job.goal contains current goal json message.
                        if job.goal is not None:

                            job_root = job.create_root(str( len(goal)-idx+1 ), job.goal,
                                                       self.controller_ns)
                            if job_root is None:
                                continue
                            rospy.loginfo("{0}: running to set up".format(len(goal)-idx+1))
                            if not job_root.setup(timeout=15):
                                rospy.logerr("{0}: failed to setup".format(len(goal)-idx+1))
                                continue
                            ## from IPython import embed; embed(); sys.exit()
                            task_node.insert_child(job_root, len(task_node.children)-idx+1)
                            break

                # remove old plans
                if len(task_node.children)>len(goal):
                    blackboard = py_trees.Blackboard()
                    edges      = blackboard.get('edges')
                    if edges is None: edges = []
                    
                    for i in range(len(task_node.children)-len(goal))[::-1]:
                        cur_edge = eval(task_node.children[i].name)
                        edges.remove(cur_edge)
                        task_node.remove_child_by_id(task_node.children[i].id)
                    blackboard.edges = edges
                        
            # Reset goals
            for job in self.jobs:
                job.goal = None
            self.current_job = job
            return 
            
                    

    def post_tick_handler(self, tree):
        """
        Check if a job is running and if it has finished. If so, prune the job subtree from the tree.
        Additionally, make a status report upon introspection of the tree.

        Args:
            tree (:class:`~py_trees.trees.BehaviourTree`): tree to investigate/manipulate.
        """
        # delete the job subtree if it is finished
        if self.busy():
            job = self.priorities.children[-2]
                        
            if job.status == py_trees.common.Status.SUCCESS or job.status == py_trees.common.Status.FAILURE or job.status == py_trees.common.Status.INVALID:
                rospy.loginfo("{0}: finished [{1}]".format(job.name, job.status))
                tree.prune_subtree(job.id)
                self.current_job = None
                
        ## # publish a status report
        ## if self.busy():
        ##     job = self.priorities.children[-2]
        ##     self.report_publisher.publish(self.current_job.create_report_string(job))
        ## elif tree.tip().has_parent_with_name("Battery Emergency"):
        ##     self.report_publisher.publish("battery")
        ## else:
            ## self.report_publisher.publish("idle")


    def busy(self):
        """
        Check if a job subtree exists and is running.

        Returns:
            :obj:`bool`: whether it is busy with a job subtree or not
        """
        return len(self.priorities.children) == 2

    @property
    def priorities(self):
        return self.tree.root.children[-1]

    def tick_tock(self):
        self.tree.tick_tock(500)

    def shutdown(self):
        self.tree.interrupt()




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
    splintered_reality = SplinteredReality(jobs=['jobs.pick_job.Move',
                                                 'jobs.place_job.Move',
                                                 'jobs.move_job.Move',
                                                 'jobs.jog_job.Move',
                                                 'jobs.gripper_job.Move',
                                                 'jobs.slide_job.Move',
                                                 'jobs.touch_job.Move'],
                                                 rec_topic_list=topic_list)
    rospy.on_shutdown(splintered_reality.shutdown)
    if not splintered_reality.setup():
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    splintered_reality.tick_tock()

    
