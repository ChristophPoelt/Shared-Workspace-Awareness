#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class DummyCarriage(Node):
    def __init__(self):
        super().__init__("demo_dummy_carriage")
        self.pub = self.create_publisher(Float64, "/elmo/id1/carriage/position/get", 10)

        # Simple motion sim
        self.pos = 0.0
        self.vel = 0.06  # m per tick
        self.dt = 0.1    # seconds
        self.min_pos = -3.0
        self.max_pos = 3.0

        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info("Publishing dummy carriage position on /elmo/id1/carriage/position/get")

    def _tick(self):
        self.pos += self.vel
        if self.pos > self.max_pos:
            self.pos = self.max_pos
            self.vel *= -1.0
        elif self.pos < self.min_pos:
            self.pos = self.min_pos
            self.vel *= -1.0

        msg = Float64()
        msg.data = float(self.pos)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = DummyCarriage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()