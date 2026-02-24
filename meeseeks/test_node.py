import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from builtin_interfaces.msg import Duration


class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')

        # Publisher for joint trajectory commands
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Action client for gripper commands
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd'
        )

        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        self.get_logger().info('Arm Controller Node started.')

    def send_joint_trajectory(self, positions: list, duration_sec: int = 10):
        """Send a joint trajectory command to the arm."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=duration_sec)

        msg.points = [point]

        self.joint_pub.publish(msg)
        self.get_logger().info(f'Published joint trajectory: {positions}')

    def send_gripper_command(self, position: float, max_effort: float = 100.0):
        """
        Send a gripper command.
        position: 0.0 = open, 0.8 = close
        """
        self.gripper_client.wait_for_server()

        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort

        self.get_logger().info(f'Sending gripper command: position={position}')
        send_goal_future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info('Gripper command completed.')


def main(args=None):
    rclpy.init(args=args)
    node = ArmController()

    # ---------------------------------------------------
    # YOUR HARDCODED SEQUENCE GOES HERE
    # Example placeholders — you will fill these in:
    # ---------------------------------------------------

    import time

    # Move to position 1
    node.send_joint_trajectory(
        positions=[-0.249, 0.176, 0.191, -1.684, 1.668, 2.064],
        duration_sec=10
    )
    time.sleep(12)  # wait longer than duration_sec to let the arm finish

    # Open gripper
    node.send_gripper_command(position=0.0)
    time.sleep(3)

    # Close gripper
    node.send_gripper_command(position=0.8)
    time.sleep(3)

    # ---------------------------------------------------

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()