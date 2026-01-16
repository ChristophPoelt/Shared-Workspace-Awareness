#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand


class RobotGestures(Node):
    """
    Unified gesture node.

    Services (std_srvs/Trigger):
      - /gesture/target_selected : CLOSE -> OPEN -> CLOSE -> OPEN (fire-and-forget)
      - /gesture/pause           : pause + close gripper
      - /gesture/resume          : resume
      - /gesture/abort           : abort + best-effort freeze (cancel gripper goals)
    """

    def __init__(self):
        super().__init__(
            "robot_gestures",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # ---------- Parameters (copied from your working old node) ----------
        self.declare_parameter("gripper_action_name", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("open_position", 0.0)
        self.declare_parameter("close_position", 0.8)
        self.declare_parameter("max_effort", 100.0)
        self.declare_parameter("pause_s", 0.5)
        self.declare_parameter("server_wait_s", 2.0)

        self._action_name = str(self.get_parameter("gripper_action_name").value)
        self._open_pos = float(self.get_parameter("open_position").value)
        self._close_pos = float(self.get_parameter("close_position").value)
        self._max_effort = float(self.get_parameter("max_effort").value)
        self._pause_s = float(self.get_parameter("pause_s").value)
        self._server_wait_s = float(self.get_parameter("server_wait_s").value)

        # ---------- State flags ----------
        self._lock = threading.Lock()
        self._busy = False
        self._paused = False
        self._abort_requested = False

        # ---------- Action client ----------
        self._gripper_client = ActionClient(self, GripperCommand, self._action_name)

        # ---------- Services ----------
        self._srv_target_selected = self.create_service(Trigger, "/gesture/target_selected", self._on_target_selected)
        self._srv_pause = self.create_service(Trigger, "/gesture/pause", self._on_pause)
        self._srv_resume = self.create_service(Trigger, "/gesture/resume", self._on_resume)
        self._srv_abort = self.create_service(Trigger, "/gesture/abort", self._on_abort)

        self.get_logger().info("RobotGestures ready.")
        self.get_logger().info(f"Gripper action: {self._action_name}")
        self.get_logger().info(
            f"open={self._open_pos}, close={self._close_pos}, effort={self._max_effort}, pause_s={self._pause_s}"
        )

    # --------------------------
    # Service callbacks
    # --------------------------

    def _on_target_selected(self, request, response):
        self.get_logger().info("SERVICE HIT: /gesture/target_selected")

        with self._lock:
            if self._busy:
                response.success = False
                response.message = "Busy: another gesture is running."
                return response
            self._busy = True
            self._paused = False
            self._abort_requested = False

        # Run in background thread
        t = threading.Thread(target=self._run_target_selected_sequence, daemon=True)
        t.start()

        response.success = True
        response.message = "Started: target_selected gesture."
        return response

    def _on_pause(self, request, response):
        self.get_logger().info("SERVICE HIT: /gesture/pause")

        with self._lock:
            self._paused = True

        # Requirement: pause command closes gripper
        ok, msg = self._close_gripper_best_effort()
        response.success = ok
        response.message = msg if msg else "Paused. Gripper close sent."
        return response

    def _on_resume(self, request, response):
        self.get_logger().info("SERVICE HIT: /gesture/resume")

        with self._lock:
            if not self._busy:
                response.success = False
                response.message = "Not busy: nothing to resume."
                return response
            self._paused = False

        response.success = True
        response.message = "Resumed."
        return response

    def _on_abort(self, request, response):
        self.get_logger().info("SERVICE HIT: /gesture/abort")

        with self._lock:
            self._abort_requested = True
            self._paused = False

        # Requirement: after abort freeze in position
        # For the gripper: best-effort cancel outstanding goals.
        # For the arm: real "hold current joints" must be implemented in your arm controller stack.
        ok, msg = self._freeze_best_effort()
        response.success = ok
        response.message = msg if msg else "Abort requested. Freeze best-effort triggered."
        return response

    # --------------------------
    # Gesture runners
    # --------------------------

    def _run_target_selected_sequence(self):
        try:
            if not self._wait_for_gripper_server():
                self.get_logger().error(f"Gripper action server not available: {self._action_name}")
                return

            self.get_logger().info("Gesture: CLOSE -> OPEN -> CLOSE -> OPEN (fire-and-forget)")

            self._check_abort_or_pause()
            self._send_no_wait(self._close_pos)
            self._sleep_with_checks(self._pause_s)

            self._check_abort_or_pause()
            self._send_no_wait(self._open_pos)
            self._sleep_with_checks(self._pause_s)

            self._check_abort_or_pause()
            self._send_no_wait(self._close_pos)
            self._sleep_with_checks(self._pause_s)

            self._check_abort_or_pause()
            self._send_no_wait(self._open_pos)

            self.get_logger().info("target_selected sequence sent (ends OPEN).")

        except RuntimeError as e:
            self.get_logger().warn(f"Gesture stopped: {e}")
        except Exception as e:
            self.get_logger().error(f"Gesture failed: {e}")
        finally:
            with self._lock:
                self._busy = False
                self._paused = False
                self._abort_requested = False

    # --------------------------
    # Helpers
    # --------------------------

    def _wait_for_gripper_server(self) -> bool:
        return self._gripper_client.wait_for_server(timeout_sec=self._server_wait_s)

    def _send_no_wait(self, position: float):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(self._max_effort)
        self.get_logger().info(f"[GRIPPER SEND] position={position}")
        self._gripper_client.send_goal_async(goal)

    def _sleep_with_checks(self, seconds: float):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self._check_abort_or_pause()
            time.sleep(0.05)

    def _check_abort_or_pause(self):
        with self._lock:
            abort = self._abort_requested
            paused = self._paused

        if abort:
            # best-effort freeze
            self._freeze_best_effort()
            raise RuntimeError("Aborted")

        # Pause loop
        while paused:
            time.sleep(0.05)
            with self._lock:
                abort = self._abort_requested
                paused = self._paused
            if abort:
                self._freeze_best_effort()
                raise RuntimeError("Aborted while paused")

    def _close_gripper_best_effort(self):
        if not self._wait_for_gripper_server():
            return False, f"Gripper action server not available: {self._action_name}"
        self._send_no_wait(self._close_pos)
        return True, "Paused. Gripper close sent."

    def _freeze_best_effort(self):
        # For gripper, best-effort is canceling goals (if any) + stop sending new commands.
        # NOTE: cancel_all_goals_async cancels goals that were accepted; fire-and-forget goals may or may not be cancelable.
        try:
            if self._wait_for_gripper_server():
                self.get_logger().warn("FREEZE(best-effort): cancel gripper goals")
                self._gripper_client._client.cancel_all_goals_async()  # internal client; works in practice
            return True, "Abort: best-effort freeze (cancel gripper goals)."
        except Exception as e:
            return False, f"Abort: freeze best-effort failed: {e}"


def main(args=None):
    rclpy.init(args=args)
    node = RobotGestures()

    # MultiThreadedExecutor so service callbacks + action client can run smoothly
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
