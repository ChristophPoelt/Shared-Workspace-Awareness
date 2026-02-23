#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class DummyServices(Node):
    def __init__(self):
        super().__init__("demo_dummy_services")

        # Robot init
        self.create_service(Trigger, "/robot/initialize", self._on_robot_init)

        # Gestures
        self.create_service(Trigger, "/gesture/target_selected", self._mk_cb("target_selected"))
        self.create_service(Trigger, "/gesture/pause", self._mk_cb("pause"))
        self.create_service(Trigger, "/gesture/resume", self._mk_cb("resume"))
        self.create_service(Trigger, "/gesture/abort", self._mk_cb("abort"))

        self.get_logger().info("Dummy services ready:")
        self.get_logger().info("  /robot/initialize")
        self.get_logger().info("  /gesture/target_selected, /gesture/pause, /gesture/resume, /gesture/abort")

    def _on_robot_init(self, request, response):
        self.get_logger().info("[DUMMY] /robot/initialize called")
        response.success = True
        response.message = "Initialized (dummy)"
        return response

    def _mk_cb(self, name: str):
        def _cb(request, response):
            self.get_logger().info(f"[DUMMY] /gesture/{name} called")
            response.success = True
            response.message = f"{name} done (dummy)"
            return response
        return _cb


def main():
    rclpy.init()
    node = DummyServices()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()