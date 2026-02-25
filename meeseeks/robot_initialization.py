#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
INIT_POSITIONS = [0.0, 0.0, 0.0, 0.0, 1.5, 0.0]


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

        self.max_joint_speed_rad_s = self._positive_param("max_joint_speed_rad_s", 0.3)
        self.min_move_duration_s = self._positive_param("min_move_duration_s", 2.0)
        self.max_move_duration_s = self._positive_param("max_move_duration_s", 12.0)
        self.fallback_move_duration_s = self._positive_param("fallback_move_duration_s", 8.0)
        self.joint_state_wait_timeout_s = self._nonnegative_param("joint_state_wait_timeout_s", 1.0)
        if self.max_move_duration_s < self.min_move_duration_s:
            self.get_logger().warn(
                "max_move_duration_s < min_move_duration_s; using min value for both bounds"
            )
            self.max_move_duration_s = self.min_move_duration_s

        self._joint_state = None
        self._joint_state_cv = threading.Condition()

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

        self.get_logger().info(
            "RobotInitializationNode ready "
            f"(speed={self.max_joint_speed_rad_s:.3f} rad/s, "
            f"duration=[{self.min_move_duration_s:.2f}, {self.max_move_duration_s:.2f}]s, "
            f"fallback={self.fallback_move_duration_s:.2f}s)"
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
                "No /joint_states received before init timeout; using fallback move duration"
            )
            return self.fallback_move_duration_s

        name_to_position = dict(zip(joint_state.name, joint_state.position))
        missing_joints = [j for j in ARM_JOINT_NAMES if j not in name_to_position]
        if missing_joints:
            self.get_logger().warn(
                f"Missing joints in /joint_states for init timing: {missing_joints}; "
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
            f"Init timing from joint delta: max_delta={max_delta:.3f} rad -> duration={duration_s:.2f}s"
        )
        return duration_s

    def _on_initialize_request(self, request, response):
        self.get_logger().info("Initializing robot...")
        _ = request

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = INIT_POSITIONS
        move_duration_s = self._compute_init_duration_s()
        set_duration_fields(point.time_from_start, move_duration_s)

        traj.points.append(point)
        self.traj_pub.publish(traj)

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
