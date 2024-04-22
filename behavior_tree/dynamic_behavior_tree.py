#!/usr/bin/env python3
import sys
import numpy as np
import json
import importlib
import argparse
import operator

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from riro_srvs.srv import StringGoalStatus
from std_msgs.msg import String
from action_msgs.msg import GoalStatus

import py_trees
import py_trees_ros
import py_trees.console as console
from py_trees_ros import exceptions, utilities

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from .subtrees import WM2Blackboard
from .subtrees import Grnd2Blackboard
## from .subtrees import Status2Blackboard
from . import decorators
from py_trees_ros.subscribers import ToBlackboard 


def create_root():
    """
    Create a basic tree and start a 'Topics2BB' work sequence that
    takes the asynchronicity out of subscription.
    
    Returns:
    :class:`~py_trees.behaviour.Behaviour`: the root of the tree
    """
    root = py_trees.composites.Parallel(
        name="ROOT",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(
            synchronise=False )
        )
    
    # ---------------- Root->Blackboard ------------------------
    grnd2bb = Grnd2Blackboard.ToBlackboard(name="Grnd2BB",
                                           topic_name="symbol_grounding")
    # wm2bb = WM2Blackboard.ToBlackboard(name="WM2BB", topic_name="/world_model")
    status2bb = ToBlackboard(name="Status2BB",
                             topic_name="arm_client/goal_status",
                             topic_type=GoalStatus,
                             blackboard_variables={"goal_id": "goal_info.goal_id.uuid", "goal_status": "status"},
                             qos_profile=py_trees_ros.utilities.qos_profile_unlatched()
)
    # ---------------- Root->Priorities- -----------------------
    priorities = py_trees.composites.Selector("Priorities",
                                              memory=False)
    idle       = py_trees.behaviours.Running(name="Idle")
    priorities.add_child(idle)
    
    root.add_children([grnd2bb, status2bb, priorities])
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
            

class SplinteredReality(Node):

    def __init__(self, jobs, rec_topic_list=None, n_loop=1,
                 enable_inf_loop=False, loop_timeout=-1):
        """
        Initialise a core tree (minus a job) and preload job classes ready to
         be used in spinning up and running the job later when requested.

        Args:
            jobs ([:obj:`str`]): list of module names as strings (e.g. 'py_trees_ros.tutorials.jobs.Scan')
        """
        super().__init__('tree')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ("sim", Parameter.Type.BOOL),
                ("gripper_open_pos", Parameter.Type.DOUBLE),
                ("gripper_close_pos", Parameter.Type.DOUBLE),
                ("gripper_open_force", Parameter.Type.DOUBLE),
                ("gripper_close_force", Parameter.Type.DOUBLE),
                ("init_config", Parameter.Type.DOUBLE_ARRAY),
                ("observe_config", Parameter.Type.DOUBLE_ARRAY),\
                ("pose_srv_channel", Parameter.Type.STRING),
                ("grasp_pose_srv_channel", Parameter.Type.STRING),
                ("height_srv_channel", Parameter.Type.STRING),
                ("rnd_pose_srv_channel", Parameter.Type.STRING),
                ("close_pose_srv_channel", Parameter.Type.STRING),
                ("world_frame", Parameter.Type.STRING),
                ("arm_base_frame", Parameter.Type.STRING),
                ("grasp_offset_z", Parameter.Type.DOUBLE),
                ("top_offset_z", Parameter.Type.DOUBLE),
                ]
        )

        self.rec_topic_list  = rec_topic_list
        
        self.n_loop          = n_loop
        self.enable_inf_loop = enable_inf_loop
        self.loop_timeout    = loop_timeout 
        
        self.blackboard            = py_trees.blackboard.Client()
        self.blackboard.register_key(key="edges", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="stop_cmd", access=py_trees.common.Access.WRITE)
        self.blackboard.edges      = None
        self.blackboard.stop_cmd   = False


        self.tree = py_trees_ros.trees.BehaviourTree(root=create_root(),
                                                     unicode_tree_debug=True)
        self.tree.add_pre_tick_handler(self.pre_tick_handler)
        self.tree.add_post_tick_handler(self.post_tick_handler)
        self.initComms()
        
        self.jobs = []
        for job in jobs:
            module_name = '.'.join(job.split('.')[:-1])
            class_name = job.split('.')[-1]
            self.jobs.append(getattr(importlib.import_module("behavior_tree."+module_name), class_name)(self))
        self.current_job = None
        console.loginfo("dynamic_behavior_tree: initialized")

        
    def initComms(self):
        """Initialize communications"""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,           
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10
        )
         
        # Shared services
        self.action_client = self.create_client(StringGoalStatus, 'arm_client/command',
                                        qos_profile=qos_profile)
                                        ## qos_profile=rclpy.qos.qos_profile_services_default)
        if not self.action_client.wait_for_service(timeout_sec=3.0):
            raise exceptions.TimedOutError('command service not available, waiting again...')

        # get odom 2 base
        
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer,
                                             node=self,
                                             spin_thread=True,
                                             qos=qos_profile,
                                                 )

    def setup(self):
        """
        Redirect the setup function"

        Returns:
            :obj:`bool`: whether it timed out trying to setup
        """
        try:
            self.tree.setup(node=self, timeout=15.0)
        except py_trees_ros.exceptions.TimedOutError as e:
            console.logerror("failed to setup the tree, aborting [{}]".format(str(e)))
            self.tree.shutdown()
            return False
        return True

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
            cancel_seq = py_trees.composites.Sequence(name="Cancel", memory=True)        
            is_stop_requested = py_trees.behaviours.CheckBlackboardVariableValue(
                name="Stop?",
                check=py_trees.common.ComparisonExpression(
                    variable="stop_cmd",
                    value=True,
                    operator=operator.eq
                    )
                )
            cancel_seq.add_child(is_stop_requested)                                
            task_list = []

            # self.jobs are holding all available job classes
            for idx in range(len(goal)):                
                for job in self.jobs:
                    # job.goal contains current goal json message.
                    if job.goal is not None:
                        job_root = job.create_root(self.action_client,
                                                   str(idx+1),
                                                   goal=job.goal,
                                                   tf_buffer=self.tf_buffer,
                                                   rec_topic_list=self.rec_topic_list)
                        if job_root is None:
                            continue
                        console.loginfo("{0}: pre_tick_handler running to set up all subtree modules".format(idx+1))
                        
                        try:
                            py_trees.trees.setup(
                                root=job_root,
                                node=self)
                                #job_root.setup(timeout=15)
                        except RuntimeError as e:
                            console.logerror("RuntimeError {}".format(e))
                        except Exception as e:
                            console.logerror("Exception {}".format(e))
                        ## finally:
                        ##     console.logerror("{0}: pre_tick_handler failed to setup".format(idx+1))
                            ## continue
                            
                        console.loginfo("{0}: pre_tick_handler finished setting up".format(idx+1))
                        task_list.append(job_root)
                        break

            task = py_trees.composites.Sequence(name="Task", memory=True)
            task.add_children(task_list)
                    
            if self.n_loop<=1 and self.enable_inf_loop is False:
                run_or_cancel = py_trees.composites.Selector(name="Run or Cancel?", memory=False)
                run_or_cancel.add_children([cancel_seq, task])
            else:
                loop = decorators.Loop(child=task, name='Loop', n_loop=self.n_loop,
                                       enable_inf_loop=self.enable_inf_loop,
                                       timeout=self.loop_timeout)

                run_or_cancel = py_trees.composites.Selector(name="Run or Cancel?", memory=False)
                run_or_cancel.add_children([cancel_seq, loop])
            
            root = run_or_cancel
            tree.insert_subtree(root, self.priorities.id, 0)
            console.loginfo("{0}: pre_tick_handler inserted job subtree".format(root.name))

            # Reset goals
            for job in self.jobs:
                job.goal = None
            self.current_job = job
            return

            ## if rec_job is not None:
            ##     # launch rosbag?
            ##     self.get_logger().info("Start to record rosbag")

        # Dynamic Reconfiguration
        elif self.busy() and goal is not None and False:
            # Todo find the node by name
            is_task_node = False
            for task_node in self.priorities.iterate():
                if task_node.name == 'Task':
                    is_task_node = True
                    break
            if is_task_node is False:
                console.logerror("No Task Node")
            
            # Check from the last goal
            for idx in range(1,len(goal)+1):
                if "action" not in goal[str(len(goal)-idx+1)].keys() and \
                  "primitive_action" in goal[str(len(goal)-idx+1)].keys():
                    goal[str(len(goal)-idx+1)]["action"] = goal[str(len(goal)-idx+1)]["primitive_action"]
                
                console.loginfo("Check: {}th plan - {}".format(str(len(goal)-idx)), \
                  goal[str(len(goal)-idx+1)]["action"])
                
                if len(task_node.children) >= idx and \
                  task_node.children[-idx].name == goal[str(len(goal)-idx+1)]["action"]:
                    console.loginfo("{0}: pre_tick_handler passing to set up".format(len(goal)-idx+1))
                    continue

                # check if there are removable plans
                is_rm_plan = False                
                for i in range(idx+1,len(task_node.children)+1):
                    if task_node.children[-i].name == goal[str(len(goal)-idx+1)]["action"]:
                        is_rm_plan = True
                        break

                if is_rm_plan:
                    edges = self.blackboard.get('edges')
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

                            job_root = job.create_root(self.action_client,
                                                       str( len(goal)-idx+1 ),
                                                       goal=job.goal)
                            if job_root is None:
                                continue
                            console.loginfo("{0}: pre_tick_handler created a job root".format(len(goal)-idx+1))

                            try:
                                py_trees.trees.setup(
                                    root=job_root,
                                    node=self)
                            except Exception as e:
                                console.logerror("{0}: pre_tick_handler failed to setup".format(len(goal)-idx+1))
                                continue

                            task_node.insert_child(job_root, len(task_node.children)-idx+1)
                            console.loginfo("{0}: pre_tick_handler inserted child".format(len(goal)-idx+1))
                            break

                # remove old plans
                if len(task_node.children)>len(goal):
                    edges = self.blackboard.get('edges')
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
                console.loginfo("{0}: post_tick_handler finished [{1}]".format(job.name, job.status))
                tree.prune_subtree(job.id)
                self.current_job = None

                
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

    def run(self):
        number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
        self.tree.tick_tock_count = 0
        
        start_time = self.get_clock().now()
        while rclpy.ok():

            rclpy.spin_once(self, timeout_sec=0)
            self.tree.tick(self.pre_tick_handler, self.post_tick_handler)
            #self.tree.tick_tock_count += 1
            
            # rate.sleep sleeps forever. So manually implemented..
            while rclpy.ok():
                time_now = self.get_clock().now()
                if time_now - start_time > rclpy.duration.Duration(nanoseconds=5e+8):
                    start_time = time_now
                    break
                rclpy.spin_once(self, timeout_sec=0)
        
        
    def shutdown(self):
        self.tree.interrupt()
        self.tree.shutdown()




##############################################################################
# Main
##############################################################################
def main(args=None):

    args = get_args(sysargv=sys.argv)[0]

    rclpy.init()
    
    # load the list of topics for recording
    if args.topic_json is None or args.topic_json.find('None')>=0:
        topic_list = None
    else:
        topic_list = load_topic_list(args.topic_json)
    py_trees.logging.level = py_trees.logging.Level.DEBUG

    # TODO: import a list of jobs from a json file or ROS parameter server.
    # Keep the default job on the top
    splintered_reality = SplinteredReality(jobs=['jobs.pick_job.Move',
                                                 'jobs.place_job.Move',
                                                 'jobs.move_job.Move',
                                                 ## ## 'jobs.handover_job.Move',
                                                 ## ## 'jobs.jog_job.Move',
                                                 'jobs.gripper_job.Move',
                                                 'jobs.observe_job.Move',
                                                 'jobs.artagobserve_job.Move',
                                                 'jobs.ssdmove_job.Move',
                                                 'jobs.ssdremove_job.Move',
                                                 'jobs.ssdreinsert_job.Move',
                                                 'jobs.ssdmove_resil_job.Move',
                                                 ## ## 'jobs.slide_job.Move',
                                                 ## ## 'jobs.attach_job.Move',
                                                 ## ## 'jobs.touch_job.Move'
                                                     ],
                                                 rec_topic_list=topic_list)
    rclpy.get_default_context().on_shutdown(splintered_reality.shutdown)
    if not splintered_reality.setup():
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)


    splintered_reality.run()
    ## splintered_reality.tick_tock()
    ## #console.loginfo("{}".format(splintered_reality.tree.count))

    ## executor = MultiThreadedExecutor()    
    ## try:
    ##     rclpy.spin(splintered_reality, executor)
    ## except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    ##     pass
    
    splintered_reality.shutdown()
    rclpy.try_shutdown()
    
    
def get_args(sysargv=None):
    p = argparse.ArgumentParser(
        description="Run a behavior tree")
    p.add_argument('--viz', action='store_true', dest='viz',
                 default=False,
                 help='use visualization code for rviz')
    p.add_argument('--rec_topics', action='store', dest='topic_json',
                 default=None, help='a list of topic to record')
    argv = sysargv[1:] if sysargv is not None else sys.argv[1:]
    args = p.parse_known_args(argv)
    return args

    
if __name__ == '__main__':
    main()
