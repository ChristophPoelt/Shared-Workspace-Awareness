import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import GripperCommand


class MockArmSubscriber(Node):
    def __init__(self):
        super().__init__('mock_arm_subscriber')

        # Subscribe to joint trajectory topic
        self.joint_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.joint_callback,
            10
        )

        # Mock action server for gripper
        self.gripper_server = ActionServer(
            self,
            GripperCommand,
            '/robotiq_gripper_controller/gripper_cmd',
            self.gripper_callback
        )

        self.get_logger().info('Mock Arm Subscriber is running and listening...')

    def joint_callback(self, msg):
        self.get_logger().info('--- Joint Trajectory Received ---')
        self.get_logger().info(f'  Joints : {msg.joint_names}')
        for i, point in enumerate(msg.points):
            self.get_logger().info(f'  Point {i}: positions={[round(p, 4) for p in point.positions]}')
            self.get_logger().info(f'           time_from_start={point.time_from_start.sec}s')

    async def gripper_callback(self, goal_handle):
        self.get_logger().info('--- Gripper Command Received ---')
        position = goal_handle.request.command.position
        effort = goal_handle.request.command.max_effort
        state = 'OPEN' if position == 0.0 else 'CLOSE' if position >= 0.8 else f'position={position}'
        self.get_logger().info(f'  Gripper : {state} (position={position}, max_effort={effort})')

        goal_handle.succeed()

        result = GripperCommand.Result()
        result.position = position
        result.effort = effort
        result.stalled = False
        result.reached_goal = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockArmSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()