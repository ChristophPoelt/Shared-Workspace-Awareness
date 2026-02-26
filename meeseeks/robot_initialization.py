#!/usr/bin/env python3
import os
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
INIT_POSITIONS = [0.002, -0.8, -1.38, -0.06, -1.0, -1.55]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def set_duration_fields(duration_msg, duration_s: float) -> None:
    duration_s = max(0.0, float(duration_s))
    sec = int(duration_s)
    nanosec = int(round((duration_s - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    duration_msg.sec = sec
    duration_msg.nanosec = nanosec


class RobotInitializationNode(Node):
    def __init__(self):
        super().__init__("robot_initialization")
        self.declare_parameter("max_joint_speed_rad_s", 0.3)
        self.declare_parameter("min_move_duration_s", 2.0)
        self.declare_parameter("max_move_duration_s", 12.0)
        self.declare_parameter("fallback_move_duration_s", 8.0)
        self.declare_parameter("joint_state_wait_timeout_s", 1.0)
        self.declare_parameter("publish_gating_topics", False)
        self.declare_parameter("ready_publish_delay_s", 0.2)

        self.max_joint_speed_rad_s = self._positive_param("max_joint_speed_rad_s", 0.3)
        self.min_move_duration_s = self._positive_param("min_move_duration_s", 2.0)
        self.max_move_duration_s = self._positive_param("max_move_duration_s", 12.0)
        self.fallback_move_duration_s = self._positive_param("fallback_move_duration_s", 8.0)
        self.joint_state_wait_timeout_s = self._nonnegative_param("joint_state_wait_timeout_s", 5.0)
        self.publish_gating_topics = bool(self.get_parameter("publish_gating_topics").value)
        self.ready_publish_delay_s = self._nonnegative_param("ready_publish_delay_s", 0.2)
        if self.max_move_duration_s < self.min_move_duration_s:
            self.get_logger().warn(
                "max_move_duration_s < min_move_duration_s; using min value for both bounds"
            )
            self.max_move_duration_s = self.min_move_duration_s

        self._joint_state = None
        self._joint_state_cv = threading.Condition()
        self._ready_timer = None
        self._last_init_duration_s = None
        self._state = "initializing"
        self._arm_armed = False

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)

        self.create_service(
            Trigger,
            "/robot/initialize",
            self._on_initialize_request,
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        self._state_pub = None
        self._arm_pub = None
        if self.publish_gating_topics:
            state_qos = QoSProfile(depth=1)
            state_qos.reliability = ReliabilityPolicy.RELIABLE
            state_qos.durability = DurabilityPolicy.VOLATILE
            self._state_pub = self.create_publisher(String, "/robot_control_state", state_qos)
            self._arm_pub = self.create_publisher(Bool, "/arm_armed", state_qos)
            self._publish_gate_topics()
            self.get_logger().warn(
                "[GATE] publish_gating_topics=true. Use only for standalone/demo initialization; "
                "main_logic should be the canonical state publisher in full bringup."
            )

        self.create_timer(5.0, self._warn_on_duplicate_node_names)

        self.get_logger().info(
            f"[INIT] node={self.get_name()} pid={os.getpid()} RobotInitializationNode ready "
            f"(speed={self.max_joint_speed_rad_s:.3f} rad/s, "
            f"duration=[{self.min_move_duration_s:.2f}, {self.max_move_duration_s:.2f}]s, "
            f"fallback={self.fallback_move_duration_s:.2f}s)"
        )
        self.get_logger().info(
            "[INIT] /robot/initialize moves arm to folded/safe pose via /joint_trajectory_controller/joint_trajectory"
        )

    def _positive_param(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warn(f"Parameter {name} must be > 0. Using default {default}.")
            return default
        return value

    def _nonnegative_param(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            self.get_logger().warn(f"Parameter {name} must be >= 0. Using default {default}.")
            return default
        return value

    def _on_joint_state(self, msg: JointState) -> None:
        with self._joint_state_cv:
            self._joint_state = msg
            self._joint_state_cv.notify_all()

    def _wait_for_joint_state(self, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        with self._joint_state_cv:
            while self._joint_state is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._joint_state_cv.wait(timeout=remaining)
            return self._joint_state

    def _compute_init_duration_s(self) -> float:
        joint_state = self._wait_for_joint_state(self.joint_state_wait_timeout_s)
        if joint_state is None:
            self.get_logger().warn(
                "[INIT] no /joint_states received before init timeout; using fallback move duration"
            )
            return self.fallback_move_duration_s

        name_to_position = dict(zip(joint_state.name, joint_state.position))
        missing_joints = [j for j in ARM_JOINT_NAMES if j not in name_to_position]
        if missing_joints:
            self.get_logger().warn(
                f"[INIT] missing joints in /joint_states for init timing: {missing_joints}; "
                "using fallback move duration"
            )
            return self.fallback_move_duration_s

        max_delta = max(
            abs(float(target) - float(name_to_position[joint]))
            for joint, target in zip(ARM_JOINT_NAMES, INIT_POSITIONS)
        )
        duration_s = clamp(
            max_delta / self.max_joint_speed_rad_s,
            self.min_move_duration_s,
            self.max_move_duration_s,
        )
        self.get_logger().info(
            f"[INIT] timing from joint delta: max_delta={max_delta:.3f} rad -> duration={duration_s:.2f}s"
        )
        return duration_s

    def _warn_on_duplicate_node_names(self) -> None:
        try:
            names = [name for name, _ns in self.get_node_names_and_namespaces()]
        except Exception:
            return
        duplicates = sum(1 for name in names if name == self.get_name())
        if duplicates > 1:
            self.get_logger().warn(
                f"[INIT] duplicate node name detected: {self.get_name()} count={duplicates}. "
                "If a launch wrapper and standalone command are both running, stop one."
            )

    def _publish_gate_topics(self) -> None:
        if self._state_pub is None or self._arm_pub is None:
            return
        state_msg = String()
        state_msg.data = self._state
        self._state_pub.publish(state_msg)
        arm_msg = Bool()
        arm_msg.data = bool(self._arm_armed)
        self._arm_pub.publish(arm_msg)
        self.get_logger().info(
            f"[GATE] publish /robot_control_state={self._state}, /arm_armed={self._arm_armed}"
        )

    def _set_gate_state(self, state: str, arm_armed: bool) -> None:
        changed = (self._state != state) or (self._arm_armed != bool(arm_armed))
        self._state = str(state)
        self._arm_armed = bool(arm_armed)
        if changed:
            self._publish_gate_topics()

    def _schedule_ready_publish(self, move_duration_s: float, *, controllers_available: bool) -> None:
        if not self.publish_gating_topics:
            return
        if self._ready_timer is not None:
            try:
                self.destroy_timer(self._ready_timer)
            except Exception:
                pass
            self._ready_timer = None

        delay_s = max(0.0, float(move_duration_s) + self.ready_publish_delay_s)

        def _mark_ready():
            if self._ready_timer is not None:
                try:
                    self.destroy_timer(self._ready_timer)
                except Exception:
                    pass
                self._ready_timer = None
            if not controllers_available:
                self.get_logger().warn(
                    "[INIT] initialization dwell complete but no trajectory controller subscriber was detected; "
                    "keeping /arm_armed=false and /robot_control_state=initializing"
                )
                self._set_gate_state("initializing", False)
                return
            self.get_logger().info("[INIT] initialization dwell complete -> publishing ready/armed")
            self._set_gate_state("ready", True)

        self._ready_timer = self.create_timer(delay_s, _mark_ready)

    def _on_initialize_request(self, request, response):
        self.get_logger().info("[INIT] initializing robot...")
        _ = request
        self._set_gate_state("initializing", False)

        traj_subs = self.traj_pub.get_subscription_count()
        controllers_available = traj_subs > 0
        if not controllers_available:
            self.get_logger().warn(
                "[INIT] no subscribers on /joint_trajectory_controller/joint_trajectory. "
                "If running in simulation, ensure controllers are spawned/active "
                "(e.g. `ros2 control list_controllers`, spawn joint_trajectory_controller)."
            )
        else:
            self.get_logger().info(
                f"[INIT] trajectory controller subscribers detected: {traj_subs}"
            )

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = INIT_POSITIONS
        move_duration_s = self._compute_init_duration_s()
        self._last_init_duration_s = move_duration_s
        set_duration_fields(point.time_from_start, move_duration_s)

        traj.points.append(point)
        self.traj_pub.publish(traj)
        self.get_logger().info(
            f"[INIT] published safe init trajectory to /joint_trajectory_controller/joint_trajectory "
            f"(duration={move_duration_s:.2f}s)"
        )
        self._schedule_ready_publish(move_duration_s, controllers_available=controllers_available)

        response.success = True
        response.message = (
            "Robot arm moved to initial pose (carriage skipped), "
            f"duration={move_duration_s:.2f}s"
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotInitializationNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
