#!/usr/bin/env python3
"""
Target Selected Gesture (MVP version)

- Provides service: /gesture/target_selected  (std_srvs/srv/Trigger)
- When called, it performs a simple "gesture" with the gripper:
    CLOSE -> OPEN -> CLOSE -> OPEN   (ends OPEN)
- IMPORTANT: This MVP version is "fire-and-forget":
    it sends gripper action goals but does NOT wait for action results.
  This avoids the MoveIt/sim gripper result timing issues you observed.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
from rclpy.executors import MultiThreadedExecutor


class TargetSelectedGesture(Node):
    def __init__(self):
        super().__init__(
            "target_selected_gesture_node",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # Parameters (tunable for sim/real)
        self.declare_parameter("gripper_action_name", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("open_position", 0.0)   # 0.0 = open
        self.declare_parameter("close_position", 0.8)  # 0.8 = close (as in your setup)
        self.declare_parameter("max_effort", 100.0)
        self.declare_parameter("pause_s", 0.5)
        self.declare_parameter("server_wait_s", 2.0)   # how long we wait for action server on each trigger

        # Read parameters
        self._action_name = str(self.get_parameter("gripper_action_name").value)
        self._open_pos = float(self.get_parameter("open_position").value)
        self._close_pos = float(self.get_parameter("close_position").value)
        self._max_effort = float(self.get_parameter("max_effort").value)
        self._pause_s = float(self.get_parameter("pause_s").value)
        self._server_wait_s = float(self.get_parameter("server_wait_s").value)

        # Action client + Trigger service
        self._client = ActionClient(self, GripperCommand, self._action_name)
        self._srv = self.create_service(Trigger, "/gesture/target_selected", self._on_trigger)

        self.get_logger().info("Ready. Service: /gesture/target_selected")
        self.get_logger().info(f"Using gripper action: {self._action_name}")
        self.get_logger().info(
            f"open={self._open_pos}, close={self._close_pos}, effort={self._max_effort}, pause={self._pause_s}s"
        )

    def _on_trigger(self, request, response):
        self.get_logger().info("SERVICE HIT: /gesture/target_selected")

        # Make sure action server is reachable
        if not self._client.wait_for_server(timeout_sec=self._server_wait_s):
            response.success = False
            response.message = f"Gripper action server not available: {self._action_name}"
            self.get_logger().error(response.message)
            return response

        # MVP: fire-and-forget (send goals, do not wait for result)
        try:
            self.get_logger().info("Target selected gesture: CLOSE -> OPEN -> CLOSE -> OPEN (fire-and-forget)")

            self._send_no_wait(self._close_pos)
            time.sleep(self._pause_s)
            self._send_no_wait(self._open_pos)
            time.sleep(self._pause_s)
            self._send_no_wait(self._close_pos)
            time.sleep(self._pause_s)
            self._send_no_wait(self._open_pos)

            response.success = True
            response.message = "Gesture sent (fire-and-forget). Ends OPEN."
            return response

        except Exception as e:
            response.success = False
            response.message = f"Gesture failed: {e}"
            self.get_logger().error(response.message)
            return response

    def _send_no_wait(self, position: float):
        """Send a gripper goal and return immediately (do not wait for action result)."""
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self._max_effort)

        self.get_logger().info(f"[SEND] position={position}")
        self._client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSelectedGesture()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
