#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from angle_calculation import ClockwiseRail


class PointFirstJoint(Node):
    def __init__(self):
        super().__init__("point_first_joint")

        self.current_joint_state = None

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10,
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        # Instantiate rail model once
        self.rail = ClockwiseRail(
            length_long=3.43,
            length_short=0.992,
            radius=0.279,
            zero_offset=0.8,
        )

    def joint_state_cb(self, msg):
        self.current_joint_state = msg

    def point_from_rail(self, current_pos, target_pos):
        if self.current_joint_state is None:
            self.get_logger().error("No joint states received yet")
            return

        # 🔢 Compute angle using your math
        pointing_angle = self.rail.calculate_yaw_to_target(
            current_pos_raw=current_pos,
            target_pos_raw=target_pos,
        )

        self.get_logger().info(
            f"Computed pointing angle: {pointing_angle:.4f} rad"
        )

        joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        name_to_position = dict(
            zip(self.current_joint_state.name,
                self.current_joint_state.position)
        )

        # Keep current joint positions
        positions = [name_to_position[j] for j in joint_names]

        # 🎯 Move ONLY joint_1
        positions[joint_names.index("joint_1")] = pointing_angle

        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 10

        traj.points.append(point)

        self.traj_pub.publish(traj)


def main():
    rclpy.init()
    node = PointFirstJoint()

    # Wait until /joint_states is available
    while rclpy.ok() and node.current_joint_state is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Example values (same as your standalone test)
    current_pos = 4.28
    target_pos = 4.50

    node.point_from_rail(current_pos, target_pos)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
