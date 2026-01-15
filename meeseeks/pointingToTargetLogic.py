#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64

from angle_calculation import ClockwiseRail
from globalVariables import currentTargetGlobal

# Numeric positions for targets
position0 = 2.7
position1 = -2.5
position2 = -1.0

TARGET_POSITIONS = {
    "position0": position0,
    "position1": position1,
    "position2": position2,
}


class PointJoint1Node(Node):
    def __init__(self):
        super().__init__("point_joint1_node")

        self.current_joint_state = None
        self.current_rail_pos = None

        # Subscribe to joint states
        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_cb,
            10,
        )

        # Subscribe to carriage position
        self.create_subscription(
            Float64,
            "/elmo/id1/carriage/position/get",
            self.carriage_pos_cb,
            10,
        )

        # Publisher for joint trajectory
        self.traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )

        # Instantiate rail math
        self.rail = ClockwiseRail(
            length_long=3.43,
            length_short=0.992,
            radius=0.279,
            zero_offset=0.8,
        )

    # ----------------------
    # Callbacks
    # ----------------------
    def joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def carriage_pos_cb(self, msg: Float64):
        self.current_rail_pos = msg.data
        self.update_joint1()  # compute angle and send trajectory whenever we get a new position

    # ----------------------
    # Core function
    # ----------------------
    def update_joint1(self):
        # Safety checks
        if self.current_joint_state is None:
            self.get_logger().warning("No joint states received yet")
            return
        if self.current_rail_pos is None:
            self.get_logger().warning("No carriage position received yet")
            return
        if currentTargetGlobal not in TARGET_POSITIONS:
            self.get_logger().warning(f"Unknown target: {currentTargetGlobal}")
            return

        # Map target string → numeric rail position
        target_pos = TARGET_POSITIONS[currentTargetGlobal]

        # Compute yaw angle using math logic
        yaw_angle = self.rail.calculate_yaw_to_target(
            current_pos_raw=self.current_rail_pos,
            target_pos_raw=target_pos,
        )

        self.get_logger().info(
            f"Current pos: {self.current_rail_pos:.4f}, "
            f"Target pos: {target_pos:.4f}, "
            f"Yaw angle: {yaw_angle:.4f} rad"
        )

        # ----------------------
        # Publish joint trajectory (joint_1 only)
        # ----------------------
        joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        # Keep current positions of all joints
        name_to_position = dict(
            zip(self.current_joint_state.name, self.current_joint_state.position)
        )
        positions = [name_to_position[j] for j in joint_names]

        # Set joint_1 to computed yaw angle
        positions[joint_names.index("joint_1")] = yaw_angle

        # Create trajectory message
        traj = JointTrajectory()
        traj.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 10  # can be tuned

        traj.points.append(point)

        # Publish trajectory
        self.traj_pub.publish(traj)
        self.get_logger().info("Published joint_1 trajectory")


# ----------------------
# Main
# ----------------------
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
