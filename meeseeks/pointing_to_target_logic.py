#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64

from .angle_calculation import ClockwiseRail
from std_msgs.msg import String

# Numeric positions for targets
TARGET_POSITIONS = {
    "position0": 2.7,
    "position1": -2.5,
    "position2": -1.0,
}

ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


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
        self.declare_parameter("max_joint1_speed_rad_s", 0.3)
        self.declare_parameter("min_move_duration_s", 0.5)
        self.declare_parameter("max_move_duration_s", 5.0)
        self.declare_parameter("command_period_s", 0.2)
        self.declare_parameter("joint1_deadband_rad", 0.01)
        self.declare_parameter("fixed_rail_pos", 0.0)

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.max_joint1_speed_rad_s = self._positive_param("max_joint1_speed_rad_s", 0.3)
        self.min_move_duration_s = self._positive_param("min_move_duration_s", 0.5)
        self.max_move_duration_s = self._positive_param("max_move_duration_s", 5.0)
        self.command_period_s = self._positive_param("command_period_s", 0.2)
        self.joint1_deadband_rad = self._nonnegative_param("joint1_deadband_rad", 0.01)
        self.fixed_rail_pos = float(self.get_parameter("fixed_rail_pos").value)

        if self.max_move_duration_s < self.min_move_duration_s:
            self.get_logger().warn(
                "max_move_duration_s < min_move_duration_s; using min value for both bounds"
            )
            self.max_move_duration_s = self.min_move_duration_s

        self.current_target = None
        self.create_subscription(String, "/selected_target", self._on_target, 10)
        self.control_state = "ready"
        self.create_subscription(String, "/robot_control_state", self._on_control_state, state_qos)

        self.current_joint_state = None
        self.current_rail_pos = self.fixed_rail_pos
        self.has_carriage_feedback = False
        self.last_commanded_joint1 = None
        self._missing_joints_warned = False

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
            f"speed={self.max_joint1_speed_rad_s:.3f} rad/s, "
            f"duration=[{self.min_move_duration_s:.2f}, {self.max_move_duration_s:.2f}]s, "
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

    def _on_target(self, msg):
        self.current_target = msg.data
        self.get_logger().info(f"[TARGET UPDATE] {self.current_target}")

    def _on_control_state(self, msg: String):
        new_state = (msg.data or "").strip() or "ready"
        if new_state == self.control_state:
            return
        self.control_state = new_state
        self.get_logger().info(f"[CONTROL STATE] {self.control_state}")

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

    # ----------------------
    # Core logic
    # ----------------------
    def update_joint1(self):
        if self.control_state not in {"ready", "waiting_before_move"}:
            return

        if self.current_joint_state is None:
            return

        if self.current_target is None:
            return  # wait until we have a target

        if self.current_target not in TARGET_POSITIONS:
            self.get_logger().warn(f"Unknown target: {self.current_target}")
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
            return
        self._missing_joints_warned = False

        current_joint1 = float(name_to_position["joint_1"])
        if self.last_commanded_joint1 is not None:
            last_delta = abs(shortest_angle_diff(yaw_angle, self.last_commanded_joint1))
            if last_delta < self.joint1_deadband_rad:
                return

        positions = [name_to_position[j] for j in ARM_JOINT_NAMES]
        positions[0] = yaw_angle

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        delta_to_current = abs(shortest_angle_diff(yaw_angle, current_joint1))
        move_duration_s = clamp(
            delta_to_current / self.max_joint1_speed_rad_s,
            self.min_move_duration_s,
            self.max_move_duration_s,
        )
        set_duration_fields(point.time_from_start, move_duration_s)
        traj.points.append(point)

        self.traj_pub.publish(traj)
        self.last_commanded_joint1 = yaw_angle


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
