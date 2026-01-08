#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute


class StarDrawer(Node):
    def __init__(self, turtle_name: str = "bottom_left"):
        super().__init__('star_drawer')

        self.turtle_name = turtle_name

        # Publisher for velocity commands
        self.pub = self.create_publisher(
            Twist,
            f"/{self.turtle_name}/cmd_vel",
            10
        )

        # Client to teleport the turtle to the center
        self.teleport_client = self.create_client(
            TeleportAbsolute,
            f"/{self.turtle_name}/teleport_absolute"
        )

        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"Waiting for teleport service '/{self.turtle_name}/teleport_absolute'..."
            )

        self.get_logger().info(f"StarDrawer node started for turtle '{self.turtle_name}'")

    # --- helper functions -------------------------------------------------

    def center_turtle(self):
        """Teleport the turtle to the center of the turtlesim window."""
        req = TeleportAbsolute.Request()
        req.x = 5.544  # center coordinates of turtlesim
        req.y = 5.544
        req.theta = 0.0  # face right

        self.teleport_client.call_async(req)
        self.get_logger().info("Teleported turtle to center")
        time.sleep(0.5)  # short pause to let turtlesim update

    def stop_turtle(self):
        """Publish a zero Twist to stop any motion."""
        twist = Twist()
        self.pub.publish(twist)

    def move_straight(self, distance: float, speed: float = 2.0):
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

        self.stop_turtle()

    def rotate(self, angle_deg: float, angular_speed_deg: float = 120.0):
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

        self.stop_turtle()

    # --- main behaviour ---------------------------------------------------

    def draw_star(self, side_length: float = 2.5):
        """
        Draw a 5-point star.
        Each step: go straight, then turn 144° to form a star polygon.
        """
        # Start from center so we don't hit walls
        self.center_turtle()

        for _ in range(5):
            self.move_straight(side_length)
            self.rotate(-144.0)  # right turn of 144°


def main(args=None):
    rclpy.init(args=args)
    node = StarDrawer(turtle_name="bottom_left")

    try:
        # small delay just in case turtlesim is starting up
        time.sleep(1.0)
        node.draw_star()
    except KeyboardInterrupt:
        node.get_logger().info("Star drawing interrupted by user.")
    finally:
        node.stop_turtle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
