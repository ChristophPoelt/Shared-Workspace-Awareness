#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger

from meeseeks.targetSelection import selectNewTarget
import meeseeks.globalVariables as gv

class GestureController:
    """Wrapper around RobotGestures Trigger services."""

    def __init__(self, node: Node, cb_group: ReentrantCallbackGroup):
        self._node = node
        self._cli_target_selected = node.create_client(
            Trigger, "/gesture/target_selected", callback_group=cb_group
        )
        self._cli_pause = node.create_client(
            Trigger, "/gesture/pause", callback_group=cb_group
        )
        self._cli_resume = node.create_client(
            Trigger, "/gesture/resume", callback_group=cb_group
        )
        self._cli_abort = node.create_client(
            Trigger, "/gesture/abort", callback_group=cb_group
        )

    def _call_async(self, client, label: str) -> None:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().warn(f"Service not available: {label}")
                return

        fut = client.call_async(Trigger.Request())

        def _done_cb(f):
            try:
                resp = f.result()
                if resp and resp.success:
                    self._node.get_logger().info(f"{label}: OK ({resp.message})")
                elif resp:
                    self._node.get_logger().warn(f"{label}: FAIL ({resp.message})")
                else:
                    self._node.get_logger().warn(f"{label}: no response")
            except Exception as e:
                self._node.get_logger().error(f"{label}: exception: {e}")

        fut.add_done_callback(_done_cb)

    def target_selected(self) -> None:
        self._call_async(self._cli_target_selected, "/gesture/target_selected")

    def pause(self) -> None:
        self._call_async(self._cli_pause, "/gesture/pause")

    def resume(self) -> None:
        self._call_async(self._cli_resume, "/gesture/resume")

    def abort(self) -> None:
        self._call_async(self._cli_abort, "/gesture/abort")


class MainLogic(Node):
    def __init__(self):
        super().__init__("main_logic")

        self.cb_group = ReentrantCallbackGroup()
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.target_pub = self.create_publisher(String, "/selected_target", 10)
        self.control_state_pub = self.create_publisher(String, "/robot_control_state", state_qos)

        # ─── State tracking ────────────────────────────────────────────────
        self.current_carriage_pos = None
        self.target_position_threshold = 0.15  # meters
        self.state = "initializing"  # initializing, waiting_*, ready(moving), paused, aborted
        self._state_lock = threading.RLock()
        self.pre_move_wait_s = 15.0
        self.post_reach_wait_s = 15.0
        self.pause_wait_s = 30.0
        self._state_deadline = None
        self._resume_state_after_pause = None
        self._resume_state_after_abort = None
        self._suspended_deadline_remaining = None
        self._publish_control_state()

        # Service client for robot initialization (Franzi)
        self.robot_init_client = self.create_client(
            Trigger,
            "/robot/initialize",
            callback_group=self.cb_group,
        )

        # Subscribe to carriage position for target reach detection
        self.create_subscription(
            Float64,
            "/elmo/id1/carriage/position/get",
            self._on_carriage_position,
            10,
            callback_group=self.cb_group,
        )

        # Voice command subscription
        self.create_subscription(
            String,
            "/voice_commands",
            self._on_voice_command,
            10,
            callback_group=self.cb_group,
        )

        # Gesture client wrapper
        self.gesture = GestureController(self, self.cb_group)

        self.get_logger().info("=" * 60)
        self.get_logger().info("MainLogic Node Starting (manual voice input via /voice_commands)")
        self.get_logger().info("=" * 60)

        # ----- Init sequence -----
        self._initial_pose()

        # Select initial target and start the pre-movement waiting phase.
        self._start_target_cycle(initial=True)
        self.get_logger().info("Initialization complete. Workflow started.")

        # Main loop timer
        self.control_timer = self.create_timer(
            0.1, self.main_control_loop, callback_group=self.cb_group
        )

    # -------------------------
    # Voice commands
    # -------------------------
    def _on_voice_command(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        cmd = raw.lower()
        if not cmd:
            return

        self.get_logger().info(f"Voice command received: '{raw}'")

        if "abort" in cmd:
            self.abort_command_logic()
        elif "pause" in cmd:
            self.pause_command_logic()
        elif "where" in cmd and "going" in cmd:
            self.where_are_you_going_logic()
        elif "continue" in cmd:
            self.continue_logic()
        else:
            self.get_logger().warn(f"Unknown voice command: '{raw}'")

    def abort_command_logic(self) -> None:
        with self._state_lock:
            self.get_logger().info("ABORT: Stopping all operations")
            if self.state == "aborted":
                self.get_logger().warn("Robot is already aborted; waiting for continue")
                return

            self._resume_state_after_abort = self.state
            self._suspended_deadline_remaining = self._remaining_deadline_seconds()
            self._state_deadline = None
            self._set_state("aborted")
            self.gesture.abort()
            self.get_logger().info("Robot aborted and frozen (waiting for continue command)")

    def pause_command_logic(self) -> None:
        with self._state_lock:
            self.get_logger().info("PAUSE: Pausing robot movement")
            if self.state in ("waiting_before_move", "ready", "waiting_after_reach"):
                self._resume_state_after_pause = self.state
                # Pause freezes the workflow timer; resume behavior is handled in main loop.
                self._suspended_deadline_remaining = self._remaining_deadline_seconds()
                self._state_deadline = time.monotonic() + self.pause_wait_s
                self._set_state("paused")
                self.gesture.pause()
                self.get_logger().info(
                    f"Robot paused for up to {self.pause_wait_s:.0f}s "
                    "(movement will reinitiate automatically)"
                )
            else:
                self.get_logger().warn(f"Cannot pause in state: {self.state}")

    def where_are_you_going_logic(self) -> None:
        with self._state_lock:
            self.get_logger().info("WHERE ARE YOU GOING?")
            self.get_logger().info(f"  Current target: {gv.currentTargetGlobal}")
            if self.current_carriage_pos is not None:
                self.get_logger().info(f"  Current position: {self.current_carriage_pos:.3f}")
            else:
                self.get_logger().info("  Current position: unknown (no data yet)")

            # User workflow: asking this during the pre-move wait starts movement immediately.
            if self.state == "waiting_before_move":
                self.get_logger().info(
                    "WHERE-ARE-YOU-GOING command received during waiting period -> starting movement now"
                )
                self._begin_movement_to_current_target()

    def continue_logic(self) -> None:
        with self._state_lock:
            self.get_logger().info("CONTINUE: Resuming robot movement")
            if self.state == "paused":
                self._resume_from_pause(auto=False)
                self.gesture.resume()
                self.get_logger().info("Robot resumed")
            elif self.state == "aborted":
                self._resume_from_abort()
                self.get_logger().info("Robot continued after abort")
            else:
                self.get_logger().warn(f"Nothing to resume (state: {self.state})")

    # -------------------------
    # Main control loop
    # -------------------------
    def _on_carriage_position(self, msg: Float64) -> None:
        self.current_carriage_pos = msg.data

    def main_control_loop(self) -> None:
        with self._state_lock:
            if self.state == "paused":
                if self._state_deadline is not None and time.monotonic() >= self._state_deadline:
                    self.get_logger().info("Pause timeout elapsed -> reinitiating workflow/movement")
                    self._resume_from_pause(auto=True)
                    self.gesture.resume()
                return

            if self.state == "aborted":
                return

            if self.state == "waiting_before_move":
                if self._state_deadline is not None and time.monotonic() >= self._state_deadline:
                    self.get_logger().info("Pre-movement waiting period finished -> starting movement")
                    self._begin_movement_to_current_target()
                return

            if self.state == "waiting_after_reach":
                if self._state_deadline is not None and time.monotonic() >= self._state_deadline:
                    self.get_logger().info("Post-reach waiting period finished -> selecting new target")
                    self._start_target_cycle(initial=False)
                return

            if self.state != "ready":
                return

            if not self._is_target_reached():
                return

            self.get_logger().warn("=" * 60)
            self.get_logger().warn("TARGET REACHED!")
            self.get_logger().warn("=" * 60)

            self._enter_post_reach_wait()

    # ─── Target Reach Detection ────────────────────────────────────────────
    def _is_target_reached(self) -> bool:
        if self.current_carriage_pos is None:
            return False

        TARGET_POSITIONS = {
            "position0": 2.7,
            "position1": -2.5,
            "position2": -1.0,
        }

        if gv.currentTargetGlobal not in TARGET_POSITIONS:
            self.get_logger().warn(f"Unknown target: {gv.currentTargetGlobal}")
            return False

        target_pos = TARGET_POSITIONS[gv.currentTargetGlobal]
        distance = abs(self.current_carriage_pos - target_pos)
        reached = distance < self.target_position_threshold

        if distance < 0.3:
            self.get_logger().info(
                f"Target: {gv.currentTargetGlobal} ({target_pos:.3f}), "
                f"Current: {self.current_carriage_pos:.3f}, "
                f"Distance: {distance:.3f}m {'✓ REACHED' if reached else ''}"
            )

        return reached

    # -------------------------
    # Helpers
    # -------------------------
    def _select_new_target_safe(self, initial: bool) -> None:
        """Select a new target and FORCE update the global state."""
        try:
            prev = gv.currentTargetGlobal
            target = selectNewTarget(None if initial else prev)

            # Force-update global, even if selectNewTarget() only returns a value
            gv.currentTargetGlobal = target
            
            msg = String()
            msg.data = target
            self.target_pub.publish(msg)
            self.get_logger().info(f"Published /selected_target: {target}")
            self.get_logger().info(f"Selected target: {target}")

        except Exception as e:
            self.get_logger().error(f"selectNewTarget failed: {e}")
            self.get_logger().warn("Falling back to position0")
            gv.currentTargetGlobal = "position0"

    # -------------------------
    # Robot init (Franzi)
    # -------------------------
    def _initial_pose(self) -> None:
        self.get_logger().info("Requesting robot initialization...")

        if not self.robot_init_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Robot initialization service not available")
            return

        future = self.robot_init_client.call_async(Trigger.Request())

        def _done_cb(f):
            try:
                resp = f.result()
                if resp and resp.success:
                    self.get_logger().info("Robot initialized successfully")
                elif resp:
                    self.get_logger().warn(f"Robot initialization failed: {resp.message}")
                else:
                    self.get_logger().warn("Robot initialization: no response")
            except Exception as e:
                self.get_logger().error(f"Initialization service error: {e}")

        future.add_done_callback(_done_cb)

    # Pointing node handles moving joint_1 toward the current target
    def _target_indication(self) -> None:
        self.get_logger().info(
            "Pointing gesture is handled by the separate pointing_to_target_logic node"
        )

    def _publish_control_state(self) -> None:
        msg = String()
        msg.data = self.state
        self.control_state_pub.publish(msg)

    def _set_state(self, new_state: str) -> None:
        if self.state == new_state:
            return
        self.state = new_state
        self._publish_control_state()

    def _remaining_deadline_seconds(self):
        if self._state_deadline is None:
            return None
        return max(0.0, self._state_deadline - time.monotonic())

    def _start_target_cycle(self, initial: bool) -> None:
        self._select_new_target_safe(initial=initial)
        if initial:
            self.get_logger().info(f"Initial target selected: {gv.currentTargetGlobal}")
        else:
            self.get_logger().info(f"New target selected: {gv.currentTargetGlobal}")

        # Target selected gesture, then target indication (pointing node reacts to the target topic).
        self.gesture.target_selected()
        self._target_indication()

        self._state_deadline = time.monotonic() + self.pre_move_wait_s
        self._set_state("waiting_before_move")
        self.get_logger().info(
            f"Waiting {self.pre_move_wait_s:.0f}s before moving to {gv.currentTargetGlobal}"
        )

    def _begin_movement_to_current_target(self) -> None:
        self._state_deadline = None
        self._set_state("ready")
        self.get_logger().info(f"Moving toward target: {gv.currentTargetGlobal}")

    def _enter_post_reach_wait(self) -> None:
        self._state_deadline = time.monotonic() + self.post_reach_wait_s
        self._set_state("waiting_after_reach")
        self.get_logger().info(
            f"Waiting {self.post_reach_wait_s:.0f}s before selecting the next target"
        )

    def _resume_from_pause(self, auto: bool) -> None:
        prev_state = self._resume_state_after_pause or "ready"
        remaining = self._suspended_deadline_remaining
        self._resume_state_after_pause = None
        self._suspended_deadline_remaining = None
        self._state_deadline = None

        # Workflow requirement: after pause, reinitiate movement after 30s.
        if prev_state in ("waiting_before_move", "ready"):
            self._begin_movement_to_current_target()
            if auto:
                self.get_logger().info("Pause timeout complete -> movement reinitiated")
            return

        if prev_state == "waiting_after_reach":
            self._set_state("waiting_after_reach")
            if remaining is not None:
                self._state_deadline = time.monotonic() + remaining
            else:
                self._state_deadline = time.monotonic() + self.post_reach_wait_s
            return

        self._set_state("ready")

    def _resume_from_abort(self) -> None:
        prev_state = self._resume_state_after_abort or "ready"
        remaining = self._suspended_deadline_remaining
        self._resume_state_after_abort = None
        self._suspended_deadline_remaining = None

        if prev_state == "paused":
            # Continue from an aborted-pause scenario as active movement.
            self._state_deadline = None
            self._begin_movement_to_current_target()
            return

        if prev_state in ("waiting_before_move", "waiting_after_reach"):
            self._set_state(prev_state)
            if remaining is not None:
                self._state_deadline = time.monotonic() + remaining
            else:
                wait_s = self.pre_move_wait_s if prev_state == "waiting_before_move" else self.post_reach_wait_s
                self._state_deadline = time.monotonic() + wait_s
            return

        self._state_deadline = None
        self._set_state("ready")


def main(args=None):
    rclpy.init(args=args)
    node = MainLogic()

    executor = MultiThreadedExecutor(num_threads=4)
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
