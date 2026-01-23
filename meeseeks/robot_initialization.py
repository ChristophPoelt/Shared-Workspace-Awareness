#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32


class RobotInitializationNode(Node):
    def __init__(self):
        super().__init__("robot_initialization")

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

        self.carriage_pub = self.create_publisher(
            Float32,
            "/elmo/id1/carriage/position/set",
            10,
        )

        self.get_logger().info("RobotInitializationNode ready")

    def _on_initialize_request(self, request, response):
        self.get_logger().info("Initializing robot...")

        traj = JointTrajectory()
        traj.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 1.5708, 0.0, 0.0, 0.0]
        point.time_from_start.sec = 5

        traj.points.append(point)
        self.traj_pub.publish(traj)

        carriage_msg = Float32()
        carriage_msg.data = 0.0
        self.carriage_pub.publish(carriage_msg)

        response.success = True
        response.message = "Robot moved to initial pose"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotInitializationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
