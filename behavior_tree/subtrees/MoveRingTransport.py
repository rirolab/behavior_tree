"""Plan and dispatch a coordinated ring transfer to both real JTC actions."""
from concurrent.futures import ThreadPoolExecutor
import time

import numpy as np
import py_trees
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from complex_action_client.ring_transport import (
    kdl_world_model, plan_ring_transport, trajectory_messages,
)


class MOVE_RING(py_trees.behaviour.Behaviour):
    """One BT behavior owns both goals, their common clock and cancellation.

    Planning uses the live starting joints and robot_description. No fallback to
    independent MOVEJ: planning, acceptance or execution failure cancels the pair.
    """
    def __init__(self, name, left_goal, right_goal, timeout=10.):
        super().__init__(name)
        self.target = np.r_[left_goal, right_goal]
        self.timeout = timeout
        self.joints = tuple(tuple(f'{prefix}_joint{i}' for i in range(1, 8))
                            for prefix in ('left_panda', 'right_fr3'))
        self.samples = {}
        self.description = None
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='ring_plan')
        self.session = None

    def setup(self, node, **kwargs):
        self.node = node
        def param(name, default):
            if not node.has_parameter(name):
                node.declare_parameter(name, default)
            return node.get_parameter(name).value
        self.sample_count = int(param('ring_transport.sample_count', 201))
        self.tolerance = float(param('ring_transport.distance_tolerance', .0001))
        self.lead = float(param('ring_transport.start_delay_sec', 2.))
        if self.lead < 1.:
            raise ValueError('ring_transport.start_delay_sec must be at least 1 s')
        self.scale = float(param('timeout_scale', 1.))
        if not np.isfinite(self.scale) or self.scale <= 0:
            raise ValueError('timeout_scale must be positive')
        self.world_frame = param('world_frame', 'base')
        names = [param('ring_transport.left_controller', '/left_panda_joint_trajectory_controller'),
                 param('ring_transport.right_controller', '/right_fr3_joint_trajectory_controller')]
        self.clients = [ActionClient(node, FollowJointTrajectory, name+'/follow_joint_trajectory') for name in names]
        self.subscriptions = [
            node.create_subscription(JointState, '/joint_states', self._state, qos_profile_sensor_data),
            node.create_subscription(String, '/robot_description', self._description,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]
        if node.has_parameter('robot_description'):
            self.description = node.get_parameter('robot_description').value

    def _description(self, message):
        self.description = message.data

    def _state(self, message):
        now = time.monotonic()
        # Replace atomically so the planning snapshot cannot see half an update.
        samples = dict(self.samples)
        for i, (name, position) in enumerate(zip(message.name, message.position)):
            velocity = message.velocity[i] if i < len(message.velocity) else None
            samples[name] = (float(position), velocity, now)
        self.samples = samples

    def _snapshot(self):
        samples, now = self.samples, time.monotonic()
        values = [samples.get(j) for arm in self.joints for j in arm]
        if any(v is None or now-v[2] > .5 for v in values):
            return None
        if any(v[1] is not None and (not np.isfinite(v[1]) or abs(v[1]) > .03) for v in values):
            return None
        q = np.array([v[0] for v in values])
        return q if np.isfinite(q).all() else None

    def initialise(self):
        self.session = dict(cancelled=False, handles=[None, None], results=[None, None],
                            errors=[], started=time.monotonic(), planning=None, sent=False)
        self.feedback_message = 'Waiting for stationary joint snapshots and robot description'

    def _plan(self, start, description):
        solvers = kdl_world_model(description, self.world_frame)
        if [s.get_joint_names() for s in solvers] != [list(j) for j in self.joints]:
            raise ValueError('Ring URDF joint order differs from controller joint order')
        return plan_ring_transport(start, self.target, [s.forward for s in solvers],
            np.concatenate([s.joint_limits_lower for s in solvers]),
            np.concatenate([s.joint_limits_upper for s in solvers]),
            np.concatenate([s.joint_velocity_limits for s in solvers]),
            duration=self.timeout/self.scale, sample_count=self.sample_count,
            distance_tolerance=self.tolerance)

    @staticmethod
    def _cancel(session):
        session['cancelled'] = True
        for handle in session['handles']:
            if handle is not None:
                handle.cancel_goal_async()
        future = session.get('planning')
        if future is not None:
            future.cancel()

    def _accepted(self, session, arm, future):
        try:
            handle = future.result()
            if not handle.accepted:
                session['errors'].append(f'arm {arm} rejected ring trajectory')
                self._cancel(session)
                return
            session['handles'][arm] = handle
            if session['cancelled']:
                handle.cancel_goal_async()
                return
            session['results'][arm] = handle.get_result_async()
        except Exception as error:
            session['errors'].append(str(error))
            self._cancel(session)

    def _failure(self, message):
        self.feedback_message = message
        self.node.get_logger().error(f'{self.name}: {message}')
        self._cancel(self.session)
        return py_trees.common.Status.FAILURE

    def update(self):
        session = self.session
        now = time.monotonic()
        if session['errors']:
            return self._failure('; '.join(session['errors']))
        if not session['sent'] and now-session['started'] > 30.:
            return self._failure('Timed out waiting for ring planning inputs/solution')
        if session['planning'] is None:
            start = self._snapshot()
            if start is None or not self.description or not all(c.server_is_ready() for c in self.clients):
                return py_trees.common.Status.RUNNING
            session['start'] = start
            session['planning'] = self.executor.submit(self._plan, start, self.description)
            self.feedback_message = 'Computing and validating coupled ring path'
            return py_trees.common.Status.RUNNING
        if not session['planning'].done():
            return py_trees.common.Status.RUNNING
        if not session['sent']:
            try:
                plan = session['planning'].result()
                latest = self._snapshot()
                if latest is None or np.max(np.abs(latest-session['start'])) > .005:
                    return self._failure('Robot moved while ring path was planned; refusing stale start')
                stamp = self.node.get_clock().now()+Duration(seconds=self.lead)
                trajectories = trajectory_messages(plan, self.joints, stamp.to_msg())
                session['stamp_ns'] = stamp.nanoseconds
                session['deadline'] = now+self.lead+float(plan.times[-1])+5.
                session['sent'] = True
                for a, trajectory in enumerate(trajectories):
                    goal = FollowJointTrajectory.Goal()
                    goal.trajectory = trajectory
                    goal.goal_time_tolerance = Duration(seconds=2.).to_msg()
                    future = self.clients[a].send_goal_async(goal)
                    future.add_done_callback(lambda f, a=a: self._accepted(session, a, f))
                self.feedback_message = f'Ring transport {plan.start_distance*1000:.1f} → {plan.end_distance*1000:.1f} mm'
            except Exception as error:
                return self._failure(str(error))
            return py_trees.common.Status.RUNNING
        clock_ns = self.node.get_clock().now().nanoseconds
        if any(h is None for h in session['handles']) and clock_ns >= session['stamp_ns']-300_000_000:
            return self._failure('Both arms did not accept before the common start deadline')
        if now > session['deadline']:
            return self._failure('Ring execution timed out')
        completed = 0
        for future in session['results']:
            if future is None or not future.done():
                continue
            try:
                result = future.result()
                if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
                    return self._failure(f'Ring controller failed: {result.result.error_string}')
                completed += 1
            except Exception as error:
                return self._failure(str(error))
        return py_trees.common.Status.SUCCESS if completed == 2 else py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.session is not None and new_status != py_trees.common.Status.SUCCESS:
            self._cancel(self.session)

    def shutdown(self):
        if self.session is not None:
            self._cancel(self.session)
        self.executor.shutdown(wait=False, cancel_futures=True)
        for client in getattr(self, 'clients', []):
            client.destroy()
        for subscription in getattr(self, 'subscriptions', []):
            self.node.destroy_subscription(subscription)
