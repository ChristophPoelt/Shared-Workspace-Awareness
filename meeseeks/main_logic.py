#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

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
        self.target_pub = self.create_publisher(String, "/selected_target", 10)

        # ─── State tracking ────────────────────────────────────────────────
        self.current_carriage_pos = None
        self.target_position_threshold = 0.15  # meters
        self.state = "initializing"  # initializing, ready, paused, aborting

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

        # Select initial target + update global
        self._select_new_target_safe(initial=True)
        self.get_logger().info(f"Initial target selected: {gv.currentTargetGlobal}")

        # Target selected gesture
        self.gesture.target_selected()

        # Target indication gesture (pointing handled elsewhere)
        self._target_indication()

        self.state = "ready"
        self.get_logger().info("Initialization complete. Robot ready.")

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
        self.get_logger().info("ABORT: Stopping all operations")
        self.state = "aborting"
        self.gesture.abort()
        self.get_logger().info("Robot aborted and frozen")
        self.state = "ready"

    def pause_command_logic(self) -> None:
        self.get_logger().info("PAUSE: Pausing robot movement")
        if self.state == "ready":
            self.state = "paused"
            self.gesture.pause()
            self.get_logger().info("Robot paused")
        else:
            self.get_logger().warn(f"Cannot pause in state: {self.state}")

    def where_are_you_going_logic(self) -> None:
        self.get_logger().info("WHERE ARE YOU GOING?")
        self.get_logger().info(f"  Current target: {gv.currentTargetGlobal}")
        if self.current_carriage_pos is not None:
            self.get_logger().info(f"  Current position: {self.current_carriage_pos:.3f}")
        else:
            self.get_logger().info("  Current position: unknown (no data yet)")

    def continue_logic(self) -> None:
        self.get_logger().info("CONTINUE: Resuming robot movement")
        if self.state == "paused":
            self.state = "ready"
            self.gesture.resume()
            self.get_logger().info("Robot resumed")
        else:
            self.get_logger().warn(f"Nothing to resume (state: {self.state})")

    # -------------------------
    # Main control loop
    # -------------------------
    def _on_carriage_position(self, msg: Float64) -> None:
        self.current_carriage_pos = msg.data

    def main_control_loop(self) -> None:
        if self.state != "ready":
            return

        if not self._is_target_reached():
            return

        self.get_logger().warn("=" * 60)
        self.get_logger().warn("TARGET REACHED!")
        self.get_logger().warn("=" * 60)

        self._select_new_target_safe(initial=False)
        self.get_logger().info(f"New target selected: {gv.currentTargetGlobal}")

        self.gesture.target_selected()
        self._target_indication()

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