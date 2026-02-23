#!/usr/bin/env python3
import time
import threading
from enum import Enum
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
from std_msgs.msg import Float64MultiArray


class BackendMode(str, Enum):
    AUTO = "auto"     # action if server exists, else sim topic
    ACTION = "action" # always action (error if server not available)
    SIM = "sim"       # always sim topic


class RobotGestures(Node):
    """
    Gesture node for Robotiq gripper.

    Services (std_srvs/Trigger):
      - /gesture/target_selected : CLOSE -> OPEN -> CLOSE -> OPEN (fire-and-forget)
      - /gesture/pause           : pause + close gripper
      - /gesture/resume          : resume
      - /gesture/abort           : abort + best-effort cancel of gripper goal
    """

    def __init__(self):
        super().__init__("robot_gestures")

        # --------------------------
        # Parameters
        # --------------------------
        self.declare_parameter("backend", BackendMode.AUTO.value)
        self.declare_parameter("server_wait_s", 0.2)

        # Action backend (real robot)
        self.declare_parameter("gripper_action_name", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("open_position", 0.0)
        self.declare_parameter("close_position", 0.8)
        self.declare_parameter("max_effort", 100.0)

        # Sim backend (topic)
        self.declare_parameter("sim_gripper_topic", "/gripper_controller/commands")

        # Gesture timing
        self.declare_parameter("pause_s", 0.5)

        # Read parameters
        backend_str = str(self.get_parameter("backend").value).strip().lower()
        self._backend_mode = BackendMode(backend_str) if backend_str in BackendMode._value2member_map_ else BackendMode.AUTO

        self._server_wait_s = float(self.get_parameter("server_wait_s").value)
        self._action_name = str(self.get_parameter("gripper_action_name").value)

        self._open_pos = float(self.get_parameter("open_position").value)
        self._close_pos = float(self.get_parameter("close_position").value)
        self._max_effort = float(self.get_parameter("max_effort").value)

        self._sim_topic = str(self.get_parameter("sim_gripper_topic").value)
        self._pause_s = float(self.get_parameter("pause_s").value)

        # --------------------------
        # State
        # --------------------------
        self._lock = threading.Lock()
        self._busy: bool = False
        self._paused: bool = False
        self._abort_requested: bool = False

        self._last_goal_handle: Optional[ClientGoalHandle] = None

        # Create QoS profile with BEST_EFFORT reliability
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        
        self._sim_pub = self.create_publisher(Float64MultiArray, self._sim_topic, qos)

        # --------------------------
        # Gripper interfaces
        # --------------------------
        self._gripper_client = ActionClient(self, GripperCommand, self._action_name)

        # Match ros2_control subscriber QoS (often BEST_EFFORT)
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self._sim_pub = self.create_publisher(Float64MultiArray, self._sim_topic, qos)

        # --------------------------
        # Services
        # --------------------------
        self.create_service(Trigger, "/gesture/target_selected", self._on_target_selected)
        self.create_service(Trigger, "/gesture/pause", self._on_pause)
        self.create_service(Trigger, "/gesture/resume", self._on_resume)
        self.create_service(Trigger, "/gesture/abort", self._on_abort)

        # --------------------------
        # Log startup
        # --------------------------
        self.get_logger().info("RobotGestures ready.")
        self.get_logger().info(f"Backend mode: {self._backend_mode.value}")
        self.get_logger().info(f"Action: {self._action_name} (open={self._open_pos}, close={self._close_pos}, effort={self._max_effort})")
        self.get_logger().info(f"Sim topic: {self._sim_topic}")

    # --------------------------
    # Backend selection
    # --------------------------
    def _select_backend(self) -> BackendMode:
        """
        Returns ACTION or SIM based on backend mode + action server availability.
        """
        if self._backend_mode == BackendMode.SIM:
            return BackendMode.SIM

        if self._backend_mode == BackendMode.ACTION:
            return BackendMode.ACTION

        # AUTO:
        if self._gripper_client.wait_for_server(timeout_sec=self._server_wait_s):
            return BackendMode.ACTION
        return BackendMode.SIM

    # --------------------------
    # Services
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

        ok, msg = self._abort_best_effort()
        response.success = ok
        response.message = msg
        return response

    # --------------------------
    # Gesture runner
    # --------------------------
    def _run_target_selected_sequence(self):
        """
        CLOSE -> OPEN -> CLOSE -> OPEN with pauses.
        Guaranteed to release busy flag.
        """
        try:
            self.get_logger().info("Gesture: CLOSE -> OPEN -> CLOSE -> OPEN")

            seq = [self._close_pos, self._open_pos, self._close_pos, self._open_pos]

            for i, pos in enumerate(seq):
                self._check_abort_or_pause()
                self._send_gripper_no_wait(pos)

                # no extra sleep after last command
                if i < len(seq) - 1:
                    self._sleep_with_checks(self._pause_s)

            self.get_logger().info("target_selected sequence done (ends OPEN).")

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
    # Sending gripper commands
    # --------------------------
    def _send_gripper_no_wait(self, position: float) -> None:
        backend = self._select_backend()
        self.get_logger().info(f"[GRIPPER] backend={backend.value}, pos={position}")

        if backend == BackendMode.ACTION:
            if not self._gripper_client.wait_for_server(timeout_sec=self._server_wait_s):
                # If user forced ACTION and it's not available, fail loudly.
                raise RuntimeError("Action backend selected but action server is not available.")

            goal = GripperCommand.Goal()
            goal.command.position = float(position)
            goal.command.max_effort = float(self._max_effort)

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

        # SIM
        msg = Float64MultiArray()
        msg.data = [float(position)]  # publish real values; sim will clamp if needed
        self._sim_pub.publish(msg)

        subs = self._sim_pub.get_subscription_count()
        self.get_logger().info(f"[GRIPPER SIM] pub {msg.data[0]} -> {self._sim_topic} (subs={subs})")

    def _close_gripper_best_effort(self) -> Tuple[bool, str]:
        try:
            self._send_gripper_no_wait(self._close_pos)
            return True, "Paused. Close command sent."
        except Exception as e:
            return False, f"Pause: failed to send close: {e}"

    def _abort_best_effort(self) -> Tuple[bool, str]:
        """
        Abort:
        - Set abort flag already done in service.
        - Best-effort cancel last action goal if we were using ACTION.
        """
        backend = self._select_backend()
        if backend != BackendMode.ACTION:
            return True, "Abort: sim backend (no goal to cancel)."

        gh = None
        with self._lock:
            gh = self._last_goal_handle

        if gh is None:
            return True, "Abort: no active gripper goal to cancel (best-effort)."

        try:
            self.get_logger().warn("Abort: canceling last gripper goal (best-effort).")
            gh.cancel_goal_async()
            return True, "Abort: cancel requested."
        except Exception as e:
            return False, f"Abort: cancel failed: {e}"

    # --------------------------
    # Pause / abort helpers
    # --------------------------
    def _sleep_with_checks(self, seconds: float, check_dt: float = 0.05):
        """
        Sleeps for `seconds` while:
          - abort stops immediately
          - pause blocks time (pause time does not consume the remaining time)
        """
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            with self._lock:
                abort = self._abort_requested
                paused = self._paused

            if abort:
                raise RuntimeError("aborted")

            if paused:
                # Wait until resume or abort (pause time should not consume remaining time)
                remaining = end - time.monotonic()
                while True:
                    time.sleep(check_dt)
                    with self._lock:
                        abort = self._abort_requested
                        paused = self._paused
                    if abort:
                        raise RuntimeError("aborted")
                    if not paused:
                        break
                end = time.monotonic() + max(0.0, remaining)
            else:
                time.sleep(check_dt)

    def _check_abort_or_pause(self, check_dt: float = 0.05):
        """
        Blocks while paused. Aborts immediately if abort is requested.
        """
        while True:
            with self._lock:
                abort = self._abort_requested
                paused = self._paused
            if abort:
                raise RuntimeError("aborted")
            if not paused:
                return
            time.sleep(check_dt)


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