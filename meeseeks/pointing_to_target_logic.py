#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, Float64

from .angle_calculation import ClockwiseRail
from std_msgs.msg import String

# Numeric positions for targets
TARGET_POSITIONS = {
    "position0": 2.7,
    "position1": -2.5,
    "position2": -1.0,
}

ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
FOLDED = [0.0, 0.0, 0.0, 0.0, 1.5, 0.0]  # full 6 joints, joint_1 overridden
# STRETCHED = [0.0, 0.3, 1.0, 0.0, 1.2, 0.0]
# STRETCHED = [0.0, 0.35, 0.60, 0.0, 1.20, 0.0]
STRETCHED = [0.0, -0.5, 0.30, 0.0, 1.10, 1.0]  # full 6 joints, joint_1 overridden
# STRETCHED = [0.0, 0.15, 0.10, 0.0, 1.00, 0.0]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def shortest_angle_diff(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


def set_duration_fields(duration_msg, duration_s: float) -> None:
    duration_s = max(0.0, float(duration_s))
    sec = int(duration_s)
    nanosec = int(round((duration_s - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    duration_msg.sec = sec
    duration_msg.nanosec = nanosec


class PointJoint1Node(Node):
    def __init__(self):
        super().__init__("point_joint1_node_logic")
        self.declare_parameter("max_joint1_speed_rad_s", 0.2)
        self.declare_parameter("min_move_duration_s", 0.5)
        self.declare_parameter("max_move_duration_s", 5.0)
        self.declare_parameter("command_period_s", 0.2)
        self.declare_parameter("joint1_deadband_rad", 0.01)
        self.declare_parameter("yaw_deadband_rad", 0.03)
        self.declare_parameter("fold_tolerance_rad", 0.05)
        self.declare_parameter("fixed_rail_pos", 0.0)
        self.declare_parameter("stretch_rate", 0.15)
        self.declare_parameter("folded_pose", FOLDED)
        self.declare_parameter("stretched_pose", STRETCHED)

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.max_joint1_speed_rad_s = self._positive_param("max_joint1_speed_rad_s", 0.3)
        self.min_move_duration_s = self._positive_param("min_move_duration_s", 0.5)
        self.max_move_duration_s = self._positive_param("max_move_duration_s", 5.0)
        self.command_period_s = self._positive_param("command_period_s", 0.2)
        self.joint1_deadband_rad = self._nonnegative_param("joint1_deadband_rad", 0.01)
        self.yaw_deadband_rad = self._nonnegative_param("yaw_deadband_rad", 0.03)
        self.fold_tolerance_rad = self._nonnegative_param("fold_tolerance_rad", 0.05)
        self.fixed_rail_pos = float(self.get_parameter("fixed_rail_pos").value)
        self.stretch_rate = self._nonnegative_param("stretch_rate", 0.15)
        self.folded_pose = self._pose_param(
            "folded_pose", FOLDED
        )
        self.stretched_pose = self._pose_param(
            "stretched_pose", STRETCHED
        )

        if self.max_move_duration_s < self.min_move_duration_s:
            self.get_logger().warn(
                "max_move_duration_s < min_move_duration_s; using min value for both bounds"
            )
            self.max_move_duration_s = self.min_move_duration_s

        self.current_target = None
        self.create_subscription(String, "/selected_target", self._on_target, target_qos)
        self.control_state = "initializing"
        self.create_subscription(String, "/robot_control_state", self._on_control_state, state_qos)
        self.arm_armed = False
        self.create_subscription(Bool, "/arm_armed", self._on_arm_armed, state_qos)

        self.current_joint_state = None
        self.current_rail_pos = self.fixed_rail_pos
        self.has_carriage_feedback = False
        self._missing_joints_warned = False
        self._last_sent_yaw = None
        self._last_sent_alpha = None
        self._last_sent_time = 0.0
        self.phase = "idle"  # idle|folding|turning|stretching
        self.alpha = 0.0
        self.stretch_active = False
        self._last_alpha_time = None
        self._last_target = None

        # Subscribe to joint states
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        # Subscribe to carriage position
        self.create_subscription(Float64, "/elmo/id1/carriage/position/get", self.carriage_pos_cb, 10)

        # Publisher for joint trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        # Rail math
        self.rail = ClockwiseRail(
            length_long=3.43,
            length_short=0.992,
            radius=0.279,
            zero_offset=0.8,
        )
        self.create_timer(self.command_period_s, self.update_joint1)
        self.get_logger().info(
            f"Pointing-only mode enabled with fixed carriage position fallback: {self.fixed_rail_pos:.3f}"
        )
        self.get_logger().info(
            "Pointing timing: "
            "safe_duration=max(2.0s, 4.0 * max_joint_delta), "
            f"extra_min_floor={self.min_move_duration_s:.2f}s, "
            f"period={self.command_period_s:.2f}s, "
            f"deadband={self.joint1_deadband_rad:.4f} rad"
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

    def _pose_param(self, name: str, default):
        value = self.get_parameter(name).value
        try:
            pose = [float(v) for v in value]
        except Exception:
            self.get_logger().warn(f"Parameter {name} must be a list of 6 floats. Using default.")
            return list(default)
        if len(pose) != 6:
            self.get_logger().warn(f"Parameter {name} must have length 6. Using default.")
            return list(default)
        return pose

    def _on_target(self, msg):
        self.current_target = msg.data
        now = self.get_clock().now().nanoseconds * 1e-9
        self.phase = "folding"
        self.alpha = 0.0
        self.stretch_active = True
        self._last_alpha_time = now
        self.get_logger().info(
            f"[TARGET UPDATE] {self.current_target} phase=folding alpha={self.alpha:.2f}"
        )

    def _on_control_state(self, msg: String):
        new_state = (msg.data or "").strip() or "ready"
        if new_state == self.control_state:
            return
        self.control_state = new_state
        if self.control_state in {"paused", "aborted"}:
            self.get_logger().info(f"[CONTROL STATE] {self.control_state}")
            self._publish_hold_position()
            self.get_logger().info("[HOLD] published")
            return
        if self.control_state == "ready":
            self.get_logger().info(f"[CONTROL STATE] ready (continuing phase={self.phase})")
            return
        self.get_logger().info(f"[CONTROL STATE] {self.control_state}")

    def _on_arm_armed(self, msg: Bool):
        new_armed = bool(msg.data)
        if new_armed == self.arm_armed:
            return
        self.arm_armed = new_armed
        self.get_logger().info(f"[ARM ARMED] {self.arm_armed}")

    # ----------------------
    # Callbacks
    # ----------------------
    def joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def carriage_pos_cb(self, msg: Float64):
        if not self.has_carriage_feedback:
            self.has_carriage_feedback = True
            self.get_logger().info("Received carriage feedback; using live carriage position for pointing")
        self.current_rail_pos = msg.data

    def _publish_hold_position(self, duration_s: float = 0.8) -> None:
        if self.current_joint_state is None:
            return

        name_to_position = dict(zip(self.current_joint_state.name, self.current_joint_state.position))
        missing_joints = [j for j in ARM_JOINT_NAMES if j not in name_to_position]
        if missing_joints:
            return

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(name_to_position[j]) for j in ARM_JOINT_NAMES]
        set_duration_fields(point.time_from_start, duration_s)
        traj.points.append(point)

        self.traj_pub.publish(traj)

    def _publish_desired_positions(
        self,
        desired_positions,
        current_positions,
        yaw_angle: float,
        now: float,
    ) -> None:
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in desired_positions]
        joint_deltas = [abs(shortest_angle_diff(point.positions[0], current_positions[0]))]
        joint_deltas.extend(
            abs(point.positions[i] - current_positions[i]) for i in range(1, len(ARM_JOINT_NAMES))
        )
        max_delta = max(joint_deltas) if joint_deltas else 0.0
        delta_to_current = joint_deltas[0] if joint_deltas else 0.0
        if delta_to_current < self.joint1_deadband_rad and max_delta < 1e-3:
            return

        move_duration_s = max(2.0, 4.0 * max_delta, self.min_move_duration_s)
        move_duration_s = min(move_duration_s, self.max_move_duration_s)

        if self._last_sent_yaw is not None:
            if (
                abs(shortest_angle_diff(yaw_angle, self._last_sent_yaw)) < 0.005
                and self._last_sent_alpha is not None
                and abs(self.alpha - self._last_sent_alpha) < 1e-3
                and (now - self._last_sent_time) < 0.5
            ):
                return

        if self.traj_pub.get_subscription_count() == 0:
            return

        set_duration_fields(point.time_from_start, move_duration_s)
        traj.points.append(point)
        self.traj_pub.publish(traj)

        self._last_sent_yaw = yaw_angle
        self._last_sent_alpha = self.alpha
        self._last_sent_time = now

    # ----------------------
    # Core logic
    # ----------------------
    def update_joint1(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.control_state != "ready":
            self._last_alpha_time = now
            return

        if not self.arm_armed:
            self._last_alpha_time = now
            return

        if self.current_joint_state is None:
            self._last_alpha_time = now
            return

        if self.current_target is None:
            self._last_alpha_time = now
            return  # wait until we have a target

        if self.current_target not in TARGET_POSITIONS:
            self.get_logger().warn(f"Unknown target: {self.current_target}")
            self._last_alpha_time = now
            return

        target_pos = TARGET_POSITIONS[self.current_target]

        yaw_angle = self.rail.calculate_yaw_to_target(
            current_pos_raw=self.current_rail_pos,
            target_pos_raw=target_pos,
        )

        # self.get_logger().info(
        #     f"Pointing: rail {self.current_rail_pos:.4f}, target {target_pos:.4f}, yaw {yaw_angle:.4f}"
        # )

        name_to_position = dict(zip(self.current_joint_state.name, self.current_joint_state.position))
        missing_joints = [j for j in ARM_JOINT_NAMES if j not in name_to_position]
        if missing_joints:
            if not self._missing_joints_warned:
                self.get_logger().warn(f"Missing joints in /joint_states: {missing_joints}")
                self._missing_joints_warned = True
            self._last_alpha_time = now
            return
        self._missing_joints_warned = False

        current_positions = [float(name_to_position[j]) for j in ARM_JOINT_NAMES]
        current_joint1 = current_positions[0]
        if self.phase == "folding":
            self.alpha = 0.0
            desired = list(current_positions)
            desired[0] = current_joint1
            desired[1:] = [float(v) for v in FOLDED[1:]]
            self._publish_desired_positions(desired, current_positions, yaw_angle, now)

            fold_error = max(
                abs(desired[i] - current_positions[i]) for i in range(1, len(ARM_JOINT_NAMES))
            )
            if fold_error < self.fold_tolerance_rad:
                self.phase = "turning"
                self.get_logger().info("fold complete -> turning")
            return

        if self.phase == "turning":
            desired = list(current_positions)
            desired[0] = yaw_angle
            desired[1:] = [float(v) for v in FOLDED[1:]]
            self._publish_desired_positions(desired, current_positions, yaw_angle, now)

            if abs(shortest_angle_diff(yaw_angle, current_joint1)) < self.yaw_deadband_rad:
                self.phase = "stretching"
                self._last_alpha_time = now
                self.get_logger().info("turn complete -> stretching")
            return

        if self.phase == "stretching":
            if self._last_alpha_time is None:
                dt = 0.0
            else:
                dt = max(0.0, now - self._last_alpha_time)
            self._last_alpha_time = now

            if self.stretch_active:
                self.alpha = min(1.0, self.alpha + self.stretch_rate * dt)
                if self.alpha >= 1.0:
                    self.alpha = 1.0
                    self.stretch_active = False

            desired = list(current_positions)
            desired[0] = yaw_angle
            for i in range(1, len(ARM_JOINT_NAMES)):
                desired[i] = (1.0 - self.alpha) * FOLDED[i] + self.alpha * STRETCHED[i]

            self._publish_desired_positions(desired, current_positions, yaw_angle, now)
            return

        # Idle/unexpected phase: hold current posture while tracking yaw only after a target arrives.
        desired = list(current_positions)
        desired[0] = yaw_angle
        self._publish_desired_positions(desired, current_positions, yaw_angle, now)


def main(args=None):
    rclpy.init(args=args)
    node = PointJoint1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
