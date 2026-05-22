from datetime import datetime
from pathlib import Path

import py_trees
import sensor_msgs.msg as sensor_msgs
import yaml
from rclpy.qos import qos_profile_sensor_data

from . import Wait

class WRITE_JOINT_STATES(py_trees.behaviour.Behaviour):
    """
    Subscribe to one joint state topic and dump the latest dual-arm snapshot to yaml.
    """

    def __init__(
        self,
        name,
        joint_states_topic="/joint_states",
        output_dir="/tmp/behavior_tree_joint_logs",
        file_name=None,
        left_robot_name="left_arm",
        right_robot_name="right_arm",
    ):
        """
        Initialise one yaml writer for the latest joint state snapshot.
        """
        super(WRITE_JOINT_STATES, self).__init__(name=name)

        # Keep static logging configuration on the behaviour instance.
        self.joint_states_topic = joint_states_topic
        self.output_dir = output_dir
        self.file_name = file_name if file_name is not None else name
        self.left_robot_name = left_robot_name
        self.right_robot_name = right_robot_name
        self.left_joint_key = "left" if "left" in left_robot_name.lower() else left_robot_name.lower()
        self.right_joint_key = "right" if "right" in right_robot_name.lower() else right_robot_name.lower()

        # Keep runtime subscription state for asynchronous updates.
        self.node = None
        self.subscription = None
        self.latest_joint_state = None
        self.session_dir = None
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="bt_start_time", access=py_trees.common.Access.READ)

    def setup(self, node):
        """
        Create the joint state subscription once during tree setup.
        """
        # Cache node and attach one sensor-data subscription.
        self.node = node
        # Resolve one shared BT session directory from blackboard start time.
        self.session_dir = self.resolve_session_dir()
        self.subscription = node.create_subscription(
            sensor_msgs.JointState,
            self.joint_states_topic,
            self._joint_states_callback,
            qos_profile_sensor_data,
        )
        self.feedback_message = f"subscribed to {self.joint_states_topic} -> {self.session_dir}"

    def initialise(self):
        """
        Reset the feedback message before one dump attempt.
        """
        # Reset only human-readable execution status.
        self.feedback_message = "waiting for joint states"

    def update(self):
        """
        Dump the latest snapshot once the topic has produced at least one message.
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)

        # Wait until at least one joint state sample has arrived.
        if self.latest_joint_state is None:
            self.feedback_message = f"waiting for {self.joint_states_topic}"
            return py_trees.common.Status.RUNNING

        # Fall back to session directory resolution if setup path was skipped.
        if self.session_dir is None:
            self.session_dir = self.resolve_session_dir()

        # Collect one sortable joint snapshot list for each robot.
        robot_joint_states = {
            self.left_robot_name: [],
            self.right_robot_name: [],
        }
        for idx, joint_name in enumerate(self.latest_joint_state.name):
            lower_joint_name = joint_name.lower()
            position = round(float(self.latest_joint_state.position[idx]), 6) if idx < len(self.latest_joint_state.position) else None
            velocity = round(float(self.latest_joint_state.velocity[idx]), 6) if idx < len(self.latest_joint_state.velocity) else None
            effort = round(float(self.latest_joint_state.effort[idx]), 6) if idx < len(self.latest_joint_state.effort) else None
            if lower_joint_name.startswith(f"{self.left_joint_key}_"):
                robot_joint_states[self.left_robot_name].append((joint_name, position, velocity, effort))
            elif lower_joint_name.startswith(f"{self.right_joint_key}_"):
                robot_joint_states[self.right_robot_name].append((joint_name, position, velocity, effort))

        # Sort robot-local joint states by joint name before yaml export.
        for robot_name in robot_joint_states:
            robot_joint_states[robot_name].sort(key=lambda joint_state: joint_state[0])

        # Pack both filtered and raw joint state data into one yaml payload.
        payload = {
            "recorded_at": datetime.now().isoformat(timespec="milliseconds"),
            "joint_states_topic": self.joint_states_topic,
            "robots": {
                self.left_robot_name: {
                    "name": [joint_state[0] for joint_state in robot_joint_states[self.left_robot_name]],
                    "position": [joint_state[1] for joint_state in robot_joint_states[self.left_robot_name]],
                    "velocity": [joint_state[2] for joint_state in robot_joint_states[self.left_robot_name]],
                    "effort": [joint_state[3] for joint_state in robot_joint_states[self.left_robot_name]],
                },
                self.right_robot_name: {
                    "name": [joint_state[0] for joint_state in robot_joint_states[self.right_robot_name]],
                    "position": [joint_state[1] for joint_state in robot_joint_states[self.right_robot_name]],
                    "velocity": [joint_state[2] for joint_state in robot_joint_states[self.right_robot_name]],
                    "effort": [joint_state[3] for joint_state in robot_joint_states[self.right_robot_name]],
                },
            },
        }

        # Write one deterministic yaml file per subtree under shared session directory.
        safe_file_name = "".join(
            char if char.isalnum() or char in ("-", "_") else "_"
            for char in self.file_name
        )
        output_path = self.session_dir / f"{safe_file_name}.yml"
        with output_path.open("w", encoding="utf-8") as output_file:
            yaml.safe_dump(payload, output_file, sort_keys=False)

        # Surface the written path through behaviour feedback.
        self.feedback_message = f"wrote {output_path}"
        return py_trees.common.Status.SUCCESS

    def resolve_session_dir(self):
        """
        Resolve one BT-scoped log directory from the shared blackboard start time.
        """
        # Prefer BT start time from blackboard so all loggers share one folder.
        try:
            bt_start_time = self.blackboard.get("bt_start_time")
        except KeyError:
            bt_start_time = None

        # Fall back to local time when BT did not publish a shared start time.
        if not bt_start_time:
            bt_start_time = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

        # Create one timestamped session directory before first file write.
        session_dir = Path(self.output_dir).expanduser() / f"bt_{bt_start_time}"
        session_dir.mkdir(parents=True, exist_ok=True)
        return session_dir

    def _joint_states_callback(self, msg):
        """
        Cache the latest joint state message for the next dump tick.
        """
        # Keep only the newest snapshot because dump is one-shot.
        self.latest_joint_state = msg


class JOINT_STATE_LOGGING(py_trees.composites.Sequence):
    """
    Wait for warmup time and then dump the latest joint state snapshot to yaml.
    """

    def __init__(
        self,
        name,
        warmup_sec=0.0,
        joint_states_topic="/joint_states",
        output_dir="/tmp/behavior_tree_joint_logs",
        left_robot_name="left_arm",
        right_robot_name="right_arm",
    ):
        """
        Initialise one logging subtree with warmup and yaml dump stages.
        """
        super(JOINT_STATE_LOGGING, self).__init__(name=name, memory=True)

        # Wait for requested warmup time before reading the latest snapshot.
        warmup_wait = Wait.WAIT(
            name=f"{name}_Warmup",
            duration=warmup_sec,
        )

        # Dump the latest joint state snapshot into one yaml file.
        write_joint_states_yaml = WRITE_JOINT_STATES(
            name=f"{name}_WriteYaml",
            joint_states_topic=joint_states_topic,
            output_dir=output_dir,
            file_name=name,
            left_robot_name=left_robot_name,
            right_robot_name=right_robot_name,
        )

        # Chain warmup and dump into one reusable subtree.
        self.add_children([warmup_wait, write_joint_states_yaml])
