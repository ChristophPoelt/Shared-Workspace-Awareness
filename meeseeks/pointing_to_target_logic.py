#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

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


class PointJoint1Node(Node):
    def __init__(self):
        super().__init__("point_joint1_node_logic")


        self.current_target = None
        self.create_subscription(String, "/selected_target", self._on_target, 10)

        self.current_joint_state = None
        self.current_rail_pos = None

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

    def _on_target(self, msg):
        self.current_target = msg.data
        self.get_logger().info(f"[TARGET UPDATE] {self.current_target}")
    # ----------------------
    # Callbacks
    # ----------------------
    def joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def carriage_pos_cb(self, msg: Float64):
        self.current_rail_pos = msg.data
        self.update_joint1()  # compute angle whenever new carriage data arrives

    # ----------------------
    # Core logic
    # ----------------------
    def update_joint1(self):
        if self.current_joint_state is None or self.current_rail_pos is None:
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

        self.get_logger().info(
            f"Pointing: rail {self.current_rail_pos:.4f}, target {target_pos:.4f}, yaw {yaw_angle:.4f}"
        )

        name_to_position = dict(zip(self.current_joint_state.name, self.current_joint_state.position))
        joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        positions = [name_to_position[j] for j in joint_names]
        positions[0] = yaw_angle

        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 1
        traj.points.append(point)

        self.traj_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = PointJoint1Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
