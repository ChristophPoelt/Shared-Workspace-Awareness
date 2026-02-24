#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64, String


class DummyCarriage(Node):
    def __init__(self):
        super().__init__("demo_dummy_carriage")
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Float64, "/elmo/id1/carriage/position/get", 10)
        self.create_subscription(String, "/robot_control_state", self._on_control_state, state_qos)

        # Simple motion sim
        self.pos = 0.0
        self.vel = 0.06  # m per tick
        self.dt = 0.1    # seconds
        self.min_pos = -3.0
        self.max_pos = 3.0
        self.control_state = "ready"

        self.timer = self.create_timer(self.dt, self._tick)
        self.get_logger().info("Publishing dummy carriage position on /elmo/id1/carriage/position/get")

    def _on_control_state(self, msg: String):
        new_state = (msg.data or "").strip() or "ready"
        if new_state == self.control_state:
            return
        self.control_state = new_state
        self.get_logger().info(f"[CONTROL STATE] {self.control_state}")

    def _tick(self):
        if self.control_state == "ready":
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
