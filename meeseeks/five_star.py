#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class StarDrawer(Node):
    def __init__(self):
        super().__init__('star_drawer')
        self.pub = self.create_publisher(Twist, '/bottom_left/cmd_vel', 10)
        self.get_logger().info("StarDrawer node started")

    def move_straight(self, distance, speed=10.0):
        """
        Move forward 'distance' meters with constant speed.
        Positive distance = forward, negative = backward.
        """
        twist = Twist()
        twist.linear.x = speed if distance >= 0 else -speed

        duration = abs(distance / speed)
        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.pub.publish(twist)
            time.sleep(0.01)

        # stop
        twist.linear.x = 0.0
        self.pub.publish(twist)

    def rotate(self, angle_deg, angular_speed_deg=60.0):
        """
        Rotate in place by 'angle_deg'.
        Positive = left (CCW), negative = right (CW).
        """
        twist = Twist()
        angular_speed = math.radians(abs(angular_speed_deg))

        # choose direction
        twist.angular.z = -angular_speed if angle_deg < 0 else angular_speed

        duration = abs(math.radians(angle_deg) / angular_speed)
        start_time = self.get_clock().now()

        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            self.pub.publish(twist)
            time.sleep(0.01)

        # stop rotation
        twist.angular.z = 0.0
        self.pub.publish(twist)

    def draw_star(self, side_length=10.0):
        """
        Draw a 5-point star using 5 equal segments.
        Each step: go straight, then turn 144° to form a star polygon.
        """
        self.move_straight(side_length)
        self.rotate(180)
        """
        for _ in range(5):
            self.move_straight(side_length)
            self.rotate(-144)   # right turn of 144°
            time.sleep(0.5)
        """


def main(args=None):
    rclpy.init(args=args)
    node = StarDrawer()

    # give turtlesim a moment to start if needed
    time.sleep(1.0)

    node.draw_star()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

