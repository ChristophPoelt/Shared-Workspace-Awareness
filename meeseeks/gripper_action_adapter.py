#!/usr/bin/env python3
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
from control_msgs.action import GripperCommand


class GripperActionAdapter(Node):
    """
    Provides /robotiq_gripper_controller/gripper_cmd action server in simulation
    by translating GripperCommand goals into /gripper_controller/commands topic commands.

    In your sim, ros2_control runs:
      - gripper_controller (JointGroupPositionController) listening on /gripper_controller/commands
      - joints: ['robotiq_85_left_knuckle_joint']  (single joint)
    but there are 0 action servers for /robotiq_gripper_controller/gripper_cmd.
    """

    def __init__(self):
        super().__init__("gripper_action_adapter")

        # Params
        self.declare_parameter("cmd_topic", "/gripper_controller/commands")
        self.declare_parameter("min_pos", 0.0)
        self.declare_parameter("max_pos", 0.8)
        self.declare_parameter("settle_time_sec", 0.2)
        self.declare_parameter("subscriber_wait_timeout_sec", 5.0)
        self.declare_parameter("subscriber_wait_poll_sec", 0.05)

        cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self._cmd_topic = cmd_topic

        # QoS to match gripper_controller subscription
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self._pub = self.create_publisher(Float64MultiArray, cmd_topic, qos)

        self._server = ActionServer(
            self,
            GripperCommand,
            "/robotiq_gripper_controller/gripper_cmd",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            f"GripperActionAdapter up. Action: /robotiq_gripper_controller/gripper_cmd -> Topic: {cmd_topic} (RELIABLE)"
        )

    def goal_callback(self, goal_request: GripperCommand.Goal) -> GoalResponse:
        wait_timeout = float(self.get_parameter("subscriber_wait_timeout_sec").value)
        if not self._wait_for_command_subscriber(wait_timeout):
            self.get_logger().warn(
                f"Rejecting gripper goal: no subscribers on {self._cmd_topic} after {wait_timeout:.2f}s"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested for gripper action goal")
        return CancelResponse.ACCEPT

    def _wait_for_command_subscriber(self, timeout_sec: float) -> bool:
        poll = max(0.01, float(self.get_parameter("subscriber_wait_poll_sec").value))
        deadline = time.monotonic() + max(0.0, timeout_sec)
        last_log_t = 0.0
        while True:
            subs = self._pub.get_subscription_count()
            if subs >= 1:
                return True

            now = time.monotonic()
            if now >= deadline:
                return False

            # Throttle logs while waiting during startup.
            if now - last_log_t > 1.0:
                self.get_logger().info(
                    f"Waiting for subscriber on {self._cmd_topic} (current={subs})"
                )
                last_log_t = now
            time.sleep(min(poll, max(0.0, deadline - now)))

    def _make_result(
        self,
        goal: GripperCommand.Goal,
        *,
        position: Optional[float] = None,
        reached_goal: bool = False,
        stalled: bool = False,
    ) -> GripperCommand.Result:
        result = GripperCommand.Result()
        result.position = float(goal.command.position if position is None else position)
        result.effort = float(goal.command.max_effort)
        result.stalled = bool(stalled)
        result.reached_goal = bool(reached_goal)
        return result

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        pos = float(goal.command.position)

        min_pos = float(self.get_parameter("min_pos").value)
        max_pos = float(self.get_parameter("max_pos").value)
        settle = float(self.get_parameter("settle_time_sec").value)
        wait_timeout = float(self.get_parameter("subscriber_wait_timeout_sec").value)

        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(goal, position=pos, reached_goal=False)

            if not self._wait_for_command_subscriber(wait_timeout):
                self.get_logger().error(
                    f"No active subscriber on {self._cmd_topic}; aborting gripper goal"
                )
                goal_handle.abort()
                return self._make_result(goal, position=pos, reached_goal=False)

            # Clamp
            if pos < min_pos:
                pos = min_pos
            elif pos > max_pos:
                pos = max_pos

            msg = Float64MultiArray()
            msg.data = [pos] 

            self._pub.publish(msg)
            self.get_logger().info(
                f"[ADAPTER] Published gripper pos {pos:.3f} to {self._cmd_topic}"
            )

            # Short cancel-aware settle loop to allow gripper_controller to react and avoid immediately reporting success while still moving.
            if settle > 0.0:
                end_t = time.monotonic() + settle
                while time.monotonic() < end_t:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        return self._make_result(goal, position=pos, reached_goal=False)
                    time.sleep(max(0.0, min(0.02, end_t - time.monotonic())))

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._make_result(goal, position=pos, reached_goal=False)

            goal_handle.succeed()
            return self._make_result(goal, position=pos, reached_goal=True)
        except Exception as exc:
            self.get_logger().error(f"Gripper action execute failed: {exc}")
            try:
                if not goal_handle.is_cancel_requested:
                    goal_handle.abort()
                else:
                    goal_handle.canceled()
            except Exception:
                pass
            return self._make_result(goal, position=pos, reached_goal=False)


def main(args=None):
    rclpy.init(args=args)
    node = GripperActionAdapter()
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
