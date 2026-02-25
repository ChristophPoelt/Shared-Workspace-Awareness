#!/usr/bin/env python3
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Float64
from std_srvs.srv import Trigger

from meeseeks.targetSelection import selectNewTarget
import meeseeks.globalVariables as gv

VOICE_COMMAND_TOPIC_DEFAULT = "/voice_commands"

class GestureController:
    """Wrapper around RobotGestures Trigger services."""

    def __init__(self, node: Node, cb_group: ReentrantCallbackGroup):
        self._node = node
        self._cli_target_selected = node.create_client(
            Trigger, "/gesture/target_selected", callback_group=cb_group
        )
        self._cli_target_selected_blocking = node.create_client(
            Trigger, "/gesture/target_selected_blocking", callback_group=cb_group
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
                self._node.get_logger().warn(f"[GESTURE] service not available: {label}")
                return

        fut = client.call_async(Trigger.Request())

        def _done_cb(f):
            try:
                resp = f.result()
                if resp and resp.success:
                    self._node.get_logger().info(f"[GESTURE] {label}: OK ({resp.message})")
                elif resp:
                    self._node.get_logger().warn(f"[GESTURE] {label}: FAIL ({resp.message})")
                else:
                    self._node.get_logger().warn(f"[GESTURE] {label}: no response")
            except Exception as e:
                self._node.get_logger().error(f"[GESTURE] {label}: exception: {e}")

        fut.add_done_callback(_done_cb)

    def target_selected(self) -> None:
        self._call_async(self._cli_target_selected, "/gesture/target_selected")

    def target_selected_blocking(self) -> bool:
        client = self._cli_target_selected_blocking
        label = "/gesture/target_selected_blocking"
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().warn(f"[GESTURE] service not available: {label}")
                return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 30.0
        while not future.done():
            if time.monotonic() >= deadline:
                self._node.get_logger().warn(f"[GESTURE] {label}: timeout waiting for response")
                return False
            time.sleep(0.05)
        try:
            resp = future.result()
        except Exception as e:
            self._node.get_logger().error(f"[GESTURE] {label}: exception: {e}")
            return False
        if resp and resp.success:
            self._node.get_logger().info(f"[GESTURE] {label}: OK ({resp.message})")
            return True
        msg = resp.message if resp else "no response"
        self._node.get_logger().warn(f"[GESTURE] {label}: FAIL ({msg})")
        return False

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
            # Command topics are operator-driven and should not require latched/transient QoS.
            durability=DurabilityPolicy.VOLATILE,
        )
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # /selected_target is also a command-style topic; prefer CLI-friendly VOLATILE QoS.
            durability=DurabilityPolicy.VOLATILE,
        )
        self.target_pub = self.create_publisher(String, "/selected_target", target_qos)
        self.control_state_pub = self.create_publisher(String, "/robot_control_state", state_qos)
        self.arm_armed_pub = self.create_publisher(Bool, "/arm_armed", state_qos)
        self.declare_parameter("voice_command_topic", VOICE_COMMAND_TOPIC_DEFAULT)
        self.declare_parameter("demo_mode", False)
        self.voice_command_topic = str(
            self.get_parameter("voice_command_topic").value or VOICE_COMMAND_TOPIC_DEFAULT
        )
        self.demo_mode = bool(self.get_parameter("demo_mode").value)
        self.declare_parameter("pointing_only_mode", True)
        self.pointing_only_mode = bool(self.get_parameter("pointing_only_mode").value)

        # ─── State tracking ────────────────────────────────────────────────
        self.current_carriage_pos = None
        self.state = "initializing"  # initializing, ready, paused, aborted
        self._state_lock = threading.RLock()
        self.pause_wait_s = 30.0
        self._state_deadline = None
        self._resume_state_after_pause = None
        self._resume_state_after_abort = None
        self._suspended_deadline_remaining = None
        self._arm_armed = False
        self._joint_states_seen = 0
        self._joint_states_required_for_arming = 2
        self._robot_init_done = False
        self._last_select_time = 0.0
        self._select_cooldown_s = 1.0
        self._pending_target = None
        self._pending_target_confirmed = False
        self._publish_control_state()
        self._publish_arm_armed()

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

        # Subscribe to joint states for startup arming (prevent arm motion until feedback is alive).
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states_for_arming,
            10,
            callback_group=self.cb_group,
        )

        # Voice command subscription
        self.create_subscription(
            String,
            self.voice_command_topic,
            self._on_voice_command,
            10,
            callback_group=self.cb_group,
        )

        # Gesture client wrapper
        self.gesture = GestureController(self, self.cb_group)

        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"[INIT] node={self.get_name()} pid={os.getpid()} "
            f"MainLogic starting (voice topic={self.voice_command_topic}, demo_mode={self.demo_mode})"
        )
        self.get_logger().info(
            "[INIT] main_logic is the canonical publisher for /robot_control_state and /arm_armed "
            "(and publishes /selected_target selections)."
        )
        self.get_logger().info(
            "[GATE] startup state=initializing arm_armed=False (strict by default until init + feedback complete)"
        )
        self.get_logger().info(
            "Voice input source is optional: run transcriber separately or use voice_cli_publisher manually."
        )
        if self.pointing_only_mode:
            self.get_logger().info(
                "Pointing-only mode enabled in main_logic: carriage reach detection is disabled."
            )
        self.get_logger().info("=" * 60)
        self.create_timer(5.0, self._warn_on_duplicate_node_names, callback_group=self.cb_group)

        # ----- Init sequence -----
        self._initial_pose()
        self.get_logger().info(
            "Initialization requested. Waiting for arm arming and a 'select' voice command."
        )

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

        self.get_logger().info(f"[INIT] voice command received: '{raw}'")

        if "abort" in cmd:
            self.abort_command_logic()
        elif "pause" in cmd:
            self.pause_command_logic()
        elif cmd in ("select", "target") or cmd.startswith("select "):
            now = time.monotonic()
            if now - self._last_select_time < self._select_cooldown_s:
                self.get_logger().info(
                    f"[TARGET] ignoring duplicate select within {self._select_cooldown_s:.1f}s cooldown"
                )
                return
            self._last_select_time = now
            self.select_target_command_logic()
        elif cmd in ("where", "confirm") or ("where" in cmd and "going" in cmd):
            self.where_are_you_going_logic()
        elif "continue" in cmd:
            self.continue_logic()
        else:
            self.get_logger().warn(f"Unknown voice command: '{raw}'")

    def select_target_command_logic(self) -> None:
        with self._state_lock:
            initial = gv.currentTargetGlobal is None
            target = self._select_new_target_safe(
                initial=initial,
                publish_selected_target=False,
            )
            if not target:
                return

            self._target_indication()
            self._pending_target = target
            self._pending_target_confirmed = False
            state_at_select = self.state
            if state_at_select == "paused":
                self.get_logger().info(
                    f"Target selected while state={self.state}; motion remains blocked until resume/continue. "
                    "Skipping target-selected gesture while blocked."
                )
                self.get_logger().info(
                    f"[TARGET] waiting for 'where are you going' confirmation before publishing target: {target}"
                )
                return

        gesture_ok = self.gesture.target_selected_blocking()
        with self._state_lock:
            if self._pending_target != target:
                self.get_logger().info(
                    f"[TARGET] select result became stale while waiting for gesture (target={target})"
                )
                return
            if not gesture_ok:
                self.get_logger().warn(
                    f"[TARGET] select gesture did not complete successfully; target remains pending: {target}"
                )
                return

            if self.state == "initializing":
                self.get_logger().info(
                    "Target selected while system is not armed yet. "
                    "Pointing will start after /arm_armed becomes True and confirmation. "
                    "Waiting for 'where are you going'."
                )
            elif self.state == "aborted":
                self.get_logger().info(
                    f"[TARGET] stored pending target while aborted: {target} "
                    "(motion will start after continue + confirmation)"
                )
            else:
                self.get_logger().info(
                    f"[TARGET] select gesture finished; waiting for 'where are you going' to start motion "
                    f"(target={gv.currentTargetGlobal})"
                )

    def abort_command_logic(self) -> None:
        with self._state_lock:
            self.get_logger().info("[GATE] ABORT requested: stopping all operations")
            if self.state == "aborted":
                self.get_logger().warn("[GATE] robot is already aborted; waiting for continue")
                return

            self._resume_state_after_abort = self.state
            self._suspended_deadline_remaining = self._remaining_deadline_seconds()
            self._state_deadline = None
            self._set_state("aborted")
            self.gesture.abort()
            self.get_logger().info("[GATE] robot aborted and frozen (waiting for continue command)")

    def pause_command_logic(self) -> None:
        with self._state_lock:
            self.get_logger().info("[GATE] PAUSE requested: pausing robot movement")
            if self.state in ("initializing", "ready"):
                self._resume_state_after_pause = self.state
                # Pause freezes any workflow deadline (currently used for pause timeout only).
                self._suspended_deadline_remaining = self._remaining_deadline_seconds()
                self._state_deadline = time.monotonic() + self.pause_wait_s
                self._set_state("paused")
                self.gesture.pause()
                self.get_logger().info(
                    f"[GATE] robot paused for up to {self.pause_wait_s:.0f}s "
                    "(movement will reinitiate automatically)"
                )
            else:
                self.get_logger().warn(f"[GATE] cannot pause in state={self.state}")

    def where_are_you_going_logic(self) -> None:
        with self._state_lock:
            self.get_logger().info("[TARGET] WHERE ARE YOU GOING?")
            self.get_logger().info(f"[TARGET] current target: {gv.currentTargetGlobal}")
            if self.current_carriage_pos is not None:
                self.get_logger().info(f"[TARGET] current position: {self.current_carriage_pos:.3f}")
            else:
                self.get_logger().info("[TARGET] current position: unknown (no data yet)")
            if self._pending_target is None:
                self.get_logger().warn("[TARGET] no pending target to confirm (say 'select' first)")
                return
            self._pending_target_confirmed = True
            self.get_logger().info(
                f"[TARGET] confirmation received for pending target: {self._pending_target}"
            )
            self._publish_pending_target_if_ready()

    def continue_logic(self) -> None:
        with self._state_lock:
            self.get_logger().info("[GATE] CONTINUE requested: resuming robot movement")
            if self.state == "paused":
                self._resume_from_pause(auto=False)
                self.gesture.resume()
                self.get_logger().info("[GATE] robot resumed")
            elif self.state == "aborted":
                self._resume_from_abort()
                self._publish_pending_target_if_ready()
                self.get_logger().info("[GATE] robot continued after abort")
            else:
                self.get_logger().warn(f"[GATE] nothing to resume (state={self.state})")

    # -------------------------
    # Main control loop
    # -------------------------
    def _on_carriage_position(self, msg: Float64) -> None:
        self.current_carriage_pos = msg.data

    def _on_joint_states_for_arming(self, _msg: JointState) -> None:
        if self._arm_armed:
            return

        self._joint_states_seen += 1
        if not self._robot_init_done:
            return
        if self._joint_states_seen < self._joint_states_required_for_arming:
            return

        self._arm_armed = True
        self._publish_arm_armed()
        self.get_logger().info(
            f"[INIT] arm armed after {self._joint_states_seen} /joint_states messages."
        )
        with self._state_lock:
            if self.state == "initializing":
                self._set_state("ready")
                self._publish_pending_target_if_ready()

    def main_control_loop(self) -> None:
        with self._state_lock:
            if self.state == "paused":
                if self._state_deadline is not None and time.monotonic() >= self._state_deadline:
                    self.get_logger().info("[GATE] pause timeout elapsed -> reinitiating workflow/movement")
                    self._resume_from_pause(auto=True)
                return

            if self.state == "aborted":
                return

            if self.state != "ready":
                return

    # -------------------------
    # Helpers
    # -------------------------
    def _select_new_target_safe(self, initial: bool, publish_selected_target: bool = True):
        """Select a new target and optionally publish /selected_target."""
        try:
            prev = gv.currentTargetGlobal
            target = selectNewTarget(None if initial else prev)

            # Force-update global, even if selectNewTarget() only returns a value
            gv.currentTargetGlobal = target
            
            self.get_logger().info(f"[TARGET] selected target: {target}")
            if publish_selected_target:
                self._publish_selected_target(target)
            else:
                self.get_logger().info(
                    f"[TARGET] not publishing /selected_target while state={self.state}; stored only"
                )
            return target

        except Exception as e:
            self.get_logger().error(f"selectNewTarget failed: {e}")
            self.get_logger().warn("[TARGET] falling back to position0")
            gv.currentTargetGlobal = "position0"
            if publish_selected_target:
                self._publish_selected_target(gv.currentTargetGlobal)
            else:
                self.get_logger().info(
                    f"[TARGET] not publishing /selected_target while state={self.state}; stored fallback only"
                )
            return gv.currentTargetGlobal

    # -------------------------
    # Robot init (Franzi)
    # -------------------------
    def _initial_pose(self) -> None:
        self.get_logger().info("[INIT] requesting /robot/initialize service...")

        if not self.robot_init_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("[INIT] /robot/initialize service not available")
            return

        future = self.robot_init_client.call_async(Trigger.Request())

        def _done_cb(f):
            try:
                resp = f.result()
                if resp and resp.success:
                    self.get_logger().info(f"[INIT] robot initialized successfully ({resp.message})")
                elif resp:
                    self.get_logger().warn(f"[INIT] robot initialization failed: {resp.message}")
                else:
                    self.get_logger().warn("[INIT] robot initialization: no response")
            except Exception as e:
                self.get_logger().error(f"[INIT] initialization service error: {e}")
            finally:
                self._robot_init_done = True

        future.add_done_callback(_done_cb)

    # Pointing node handles moving joint_1 toward the current target
    def _target_indication(self) -> None:
        self.get_logger().info(
            "[PHASE] pointing gesture is handled by the separate pointing_to_target_logic node"
        )

    def _publish_selected_target(self, target: str) -> None:
        msg = String()
        msg.data = str(target)
        self.target_pub.publish(msg)
        self.get_logger().info(f"[TARGET] published /selected_target: {target}")

    def _publish_control_state(self) -> None:
        msg = String()
        msg.data = self.state
        self.control_state_pub.publish(msg)
        self.get_logger().info(f"[GATE] publish /robot_control_state={self.state}")

    def _publish_arm_armed(self) -> None:
        msg = Bool()
        msg.data = self._arm_armed
        self.arm_armed_pub.publish(msg)
        self.get_logger().info(f"[GATE] publish /arm_armed={self._arm_armed}")

    def _set_state(self, new_state: str) -> None:
        if self.state == new_state:
            return
        old_state = self.state
        self.state = new_state
        self.get_logger().info(f"[GATE] state transition {old_state} -> {new_state}")
        self._publish_control_state()

    def _warn_on_duplicate_node_names(self) -> None:
        try:
            names = [name for name, _ns in self.get_node_names_and_namespaces()]
        except Exception:
            return
        duplicates = sum(1 for name in names if name == self.get_name())
        if duplicates > 1:
            self.get_logger().warn(
                f"[INIT] duplicate node name detected: {self.get_name()} count={duplicates}. "
                "Avoid running multiple main_logic instances; this node is the canonical state publisher."
            )

    def _remaining_deadline_seconds(self):
        if self._state_deadline is None:
            return None
        return max(0.0, self._state_deadline - time.monotonic())

    def _resume_from_pause(self, auto: bool) -> None:
        prev_state = self._resume_state_after_pause or "ready"
        remaining = self._suspended_deadline_remaining
        self._resume_state_after_pause = None
        self._suspended_deadline_remaining = None
        self._state_deadline = None

        if prev_state in ("initializing", "ready"):
            if self._arm_armed:
                self._set_state("ready")
            else:
                self._set_state("initializing")
            if auto:
                self.get_logger().info("[GATE] pause timeout complete -> motion unblocked")
            return

        if remaining is not None:
            self._state_deadline = time.monotonic() + remaining
        self._set_state("ready" if self._arm_armed else "initializing")

    def _resume_from_abort(self) -> None:
        prev_state = self._resume_state_after_abort or "ready"
        remaining = self._suspended_deadline_remaining
        self._resume_state_after_abort = None
        self._suspended_deadline_remaining = None

        self._state_deadline = None
        if remaining is not None:
            self._state_deadline = time.monotonic() + remaining

        if prev_state == "paused":
            prev_state = "ready"
        if prev_state == "initializing" and not self._arm_armed:
            self._set_state("initializing")
        else:
            self._set_state("ready" if self._arm_armed else "initializing")

    def _publish_pending_target_if_ready(self) -> None:
        if self._pending_target is None:
            return
        if not self._pending_target_confirmed:
            self.get_logger().info(
                f"[TARGET] pending target retained ({self._pending_target}); waiting for 'where are you going'"
            )
            return
        if self.state != "ready":
            self.get_logger().info(
                f"[TARGET] pending target retained ({self._pending_target}); state={self.state}"
            )
            return
        target = self._pending_target
        self._pending_target = None
        self._pending_target_confirmed = False
        self._publish_selected_target(target)
        self.get_logger().info(f"[TARGET] dispatched pending target: {target}")


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
