#!/usr/bin/env python3
import time
import threading
from enum import Enum
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
from std_msgs.msg import Float64MultiArray


class GripperBackend(Enum):
    ACTION = "action"
    SIM_TOPIC = "sim_topic"
    NONE = "none"


class RobotGestures(Node):
    """
    Unified gesture node.

    Services (std_srvs/Trigger):
      - /gesture/target_selected : CLOSE -> OPEN -> CLOSE -> OPEN (fire-and-forget)
      - /gesture/pause           : pause + close gripper
      - /gesture/resume          : resume
      - /gesture/abort           : abort + best-effort cancel of gripper goal
    """

    def __init__(self):
        super().__init__(
            "robot_gestures",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # --------------------------
        # Parameters
        # --------------------------
        # Real robot (action)
        self.declare_parameter("gripper_action_name", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("open_position", 0.0)
        self.declare_parameter("close_position", 0.8)
        self.declare_parameter("max_effort", 100.0)

        # Gesture timing
        self.declare_parameter("pause_s", 0.5)

        # How long to wait when checking action server availability (startup / periodic)
        self.declare_parameter("server_wait_s", 0.2)

        # Simulation fallback (topic)
        self.declare_parameter("sim_gripper_topic", "/gripper_controller/commands")
        # NOTE: keep these as you discovered them (your sim mapping can differ from the real robot)
        self.declare_parameter("sim_open_position", 0.6)
        self.declare_parameter("sim_close_position", 0.0)

        # Read params
        self._action_name = str(self.get_parameter("gripper_action_name").value)
        self._open_pos = float(self.get_parameter("open_position").value)
        self._close_pos = float(self.get_parameter("close_position").value)
        self._max_effort = float(self.get_parameter("max_effort").value)
        self._pause_s = float(self.get_parameter("pause_s").value)
        self._server_wait_s = float(self.get_parameter("server_wait_s").value)

        self._sim_topic = str(self.get_parameter("sim_gripper_topic").value)
        self._sim_open_pos = float(self.get_parameter("sim_open_position").value)
        self._sim_close_pos = float(self.get_parameter("sim_close_position").value)

        # --------------------------
        # State
        # --------------------------
        self._lock = threading.Lock()
        self._busy = False
        self._paused = False
        self._abort_requested = False

        # Keep last goal handle (best-effort cancel)
        self._last_goal_handle: Optional[ClientGoalHandle] = None

        # --------------------------
        # Gripper interfaces
        # --------------------------
        self._gripper_client = ActionClient(self, GripperCommand, self._action_name)
        self._sim_pub = self.create_publisher(Float64MultiArray, self._sim_topic, 10)

        # --------------------------
        # Services
        # --------------------------
        self.create_service(Trigger, "/gesture/target_selected", self._on_target_selected)
        self.create_service(Trigger, "/gesture/pause", self._on_pause)
        self.create_service(Trigger, "/gesture/resume", self._on_resume)
        self.create_service(Trigger, "/gesture/abort", self._on_abort)

        self.get_logger().info("RobotGestures ready.")
        self.get_logger().info(f"Gripper action: {self._action_name}")
        self.get_logger().info(f"Real: open={self._open_pos}, close={self._close_pos}, effort={self._max_effort}")
        self.get_logger().info(f"Sim:  open={self._sim_open_pos}, close={self._sim_close_pos}, topic={self._sim_topic}")

    # --------------------------
    # Backend selection
    # --------------------------

    def _select_backend(self) -> GripperBackend:
        # Prefer action server if available
        if self._gripper_client.wait_for_server(timeout_sec=self._server_wait_s):
            return GripperBackend.ACTION
        # Otherwise fall back to sim topic (we can publish regardless of subscriber count)
        return GripperBackend.SIM_TOPIC

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

        threading.Thread(target=self._run_target_selected_sequence, daemon=True).start()

        response.success = True
        response.message = "Started: target_selected gesture."
        return response

    def _on_pause(self, request, response):
        self.get_logger().info("SERVICE HIT: /gesture/pause")

        with self._lock:
            self._paused = True

        ok, msg = self._close_gripper_best_effort()
        response.success = ok
        response.message = msg
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

        ok, msg = self._freeze_best_effort()
        response.success = ok
        response.message = msg
        return response

    # --------------------------
    # Gesture runner
    # --------------------------

    def _run_target_selected_sequence(self):
        try:
            self.get_logger().info("Gesture: CLOSE -> OPEN -> CLOSE -> OPEN")

            self._check_abort_or_pause()
            self._send_gripper_no_wait(self._close_pos)
            self._sleep_with_checks(self._pause_s)

            self._check_abort_or_pause()
            self._send_gripper_no_wait(self._open_pos)
            self._sleep_with_checks(self._pause_s)

            self._check_abort_or_pause()
            self._send_gripper_no_wait(self._close_pos)
            self._sleep_with_checks(self._pause_s)

            self._check_abort_or_pause()
            self._send_gripper_no_wait(self._open_pos)

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
    # Gripper helpers
    # --------------------------

    def _map_real_to_sim(self, position: float) -> float:
        # Map your canonical open/close to sim-specific values
        if abs(position - self._open_pos) < 1e-6:
            return self._sim_open_pos
        if abs(position - self._close_pos) < 1e-6:
            return self._sim_close_pos
        # If you pass custom positions, forward them (use with care)
        return float(position)

    def _send_gripper_no_wait(self, position: float) -> None:
        backend = self._select_backend()

        if backend == GripperBackend.ACTION:
            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = float(self._max_effort)

            self.get_logger().info(f"[GRIPPER ACTION] position={position}")

            # Store last goal handle for best-effort cancellation
            future = self._gripper_client.send_goal_async(goal)

            def _on_goal_response(fut):
                try:
                    gh = fut.result()
                    if gh is None:
                        return
                    with self._lock:
                        self._last_goal_handle = gh
                except Exception:
                    pass

            future.add_done_callback(_on_goal_response)
            return

        if backend == GripperBackend.SIM_TOPIC:
            sim_value = self._map_real_to_sim(position)
            msg = Float64MultiArray()
            msg.data = [sim_value]
            self._sim_pub.publish(msg)

            subs = self._sim_pub.get_subscription_count()
            if subs == 0:
                self.get_logger().warn(f"[GRIPPER SIM] published value={sim_value} but subs=0 on {self._sim_topic}")
            else:
                self.get_logger().info(f"[GRIPPER SIM] value={sim_value} -> {self._sim_topic} (subs={subs})")
            return

        self.get_logger().error("[GRIPPER] No backend available")

    def _close_gripper_best_effort(self) -> Tuple[bool, str]:
        try:
            self._send_gripper_no_wait(self._close_pos)
            return True, "Paused. Gripper close sent."
        except Exception as e:
            return False, f"Pause: failed to send close: {e}"

    def _freeze_best_effort(self) -> Tuple[bool, str]:
        """
        Best-effort abort for gripper:
        - if we have a last goal handle from the action backend, request cancel
        - in sim topic backend, there's nothing to cancel; we just stop sending new commands
        """
        backend = self._select_backend()

        if backend == GripperBackend.ACTION:
            gh = None
            with self._lock:
                gh = self._last_goal_handle

            if gh is None:
                return True, "Abort: no active gripper goal to cancel (best-effort)."

            try:
                self.get_logger().warn("Abort: canceling last gripper goal (best-effort).")
                gh.cancel_goal_async()
                return True, "Abort: cancel requested for last gripper goal."
            except Exception as e:
                return False, f"Abort: cancel failed: {e}"

        # SIM_TOPIC: nothing to cancel
        return True, "Abort: sim backend (no goal to cancel)."

    # --------------------------
    # Pause/abort checks
    # --------------------------

    def _sleep_with_checks(self, seconds: float) -> None:
        t0 = time.time()
        while time.time() - t0 < seconds:
            self._check_abort_or_pause()
            time.sleep(0.05)

    def _check_abort_or_pause(self) -> None:
        with self._lock:
            abort = self._abort_requested
            paused = self._paused

        if abort:
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


def main(args=None):
    rclpy.init(args=args)
    node = RobotGestures()

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