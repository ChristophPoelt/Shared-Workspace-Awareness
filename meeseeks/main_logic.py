#!/usr/bin/env python3
import os
import re
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
DIALOGUE_WAIT_SELECT = "WAIT_SELECT"
DIALOGUE_WAIT_CONFIRM = "WAIT_CONFIRM"
DIALOGUE_WAIT_DISPATCH = "WAIT_DISPATCH"
ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

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
        self._cli_wipe = node.create_client(
            Trigger, "/gesture/wipe", callback_group=cb_group
        )
        self._cli_wipe_blocking = node.create_client(
            Trigger, "/gesture/wipe_blocking", callback_group=cb_group
        )
        self._cli_grab = node.create_client(
            Trigger, "/gesture/grab", callback_group=cb_group
        )
        self._cli_grab_blocking = node.create_client(
            Trigger, "/gesture/grab_blocking", callback_group=cb_group
        )
        self._cli_pause = node.create_client(
            Trigger, "/gesture/pause", callback_group=cb_group
        )
        self._cli_resume = node.create_client(
            Trigger, "/gesture/resume", callback_group=cb_group
        )
        self._cli_open = node.create_client(
            Trigger, "/gesture/open", callback_group=cb_group
        )
        self._cli_abort = node.create_client(
            Trigger, "/gesture/abort", callback_group=cb_group
        )

    def _call_async(self, client, label: str, wait_for_service: bool = True) -> None:
        if not client.service_is_ready():
            if not wait_for_service:
                self._node.get_logger().warn(f"[GESTURE] service not available: {label}")
                return
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
        return self._call_blocking(self._cli_target_selected_blocking, "/gesture/target_selected_blocking")

    def _call_blocking(self, client, label: str, timeout_s: float = 30.0) -> bool:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                self._node.get_logger().warn(f"[GESTURE] service not available: {label}")
                return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + max(0.5, float(timeout_s))
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

    def _call_prefer_blocking(self, blocking_client, blocking_label: str, async_client, async_label: str) -> bool:
        if blocking_client.service_is_ready():
            return self._call_blocking(blocking_client, blocking_label)
        if blocking_client.wait_for_service(timeout_sec=0.3):
            return self._call_blocking(blocking_client, blocking_label)
        self._node.get_logger().warn(
            f"[GESTURE] {blocking_label} unavailable; falling back to {async_label} + fixed wait"
        )
        self._call_async(async_client, async_label)
        time.sleep(1.5)
        return True

    def wipe_blocking_prefer(self) -> bool:
        return self._call_prefer_blocking(
            self._cli_wipe_blocking,
            "/gesture/wipe_blocking",
            self._cli_wipe,
            "/gesture/wipe",
        )

    def grab_blocking_prefer(self) -> bool:
        return self._call_prefer_blocking(
            self._cli_grab_blocking,
            "/gesture/grab_blocking",
            self._cli_grab,
            "/gesture/grab",
        )

    def pause(self) -> None:
        self._call_async(self._cli_pause, "/gesture/pause")

    def resume(self) -> None:
        self._call_async(self._cli_resume, "/gesture/resume")

    def resume_no_wait(self) -> None:
        self._call_async(self._cli_resume, "/gesture/resume", wait_for_service=False)

    def open_no_wait(self) -> None:
        self._call_async(self._cli_open, "/gesture/open", wait_for_service=False)

    def abort(self) -> None:
        self._call_async(self._cli_abort, "/gesture/abort")


class MainLogic(Node):
    def __init__(self):
        super().__init__("main_logic")

        self.cb_group = ReentrantCallbackGroup()
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
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
        self.current_arm_joint_positions = None
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
        self._arm_joint_state_valid = False
        self._last_joint_missing_signature = None
        self._robot_init_done = False
        self._last_select_time = 0.0
        self._select_cooldown_s = 1.0
        self._pending_target = None
        self._pending_target_confirmed = False
        self._pre_move_gesture_running = False
        self._next_what_gesture = "wipe"
        self._dialogue_state = DIALOGUE_WAIT_SELECT
        self._abort_auto_ready_delay_s = 0.75
        self._abort_auto_ready_timer = None
        self._publish_control_state()
        self._publish_arm_armed()

        # Service client for robot initialization
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

        # Init sequence 
        self._initial_pose()
        self.get_logger().info(
            "Initialization requested. Waiting for arm arming and a 'select' voice command."
        )

        # Main loop timer
        self.control_timer = self.create_timer(
            0.1, self.main_control_loop, callback_group=self.cb_group
        )

    # Voice commands
    def _on_voice_command(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        cmd = raw.lower()
        norm_cmd = self._normalize_voice_text(raw)
        if not cmd:
            return

        self.get_logger().info(f"[INIT] voice command received: '{raw}'")

        if "abort" in norm_cmd or "stop" in norm_cmd:
            self.abort_command_logic()
        elif "pause" in norm_cmd or "hold" in norm_cmd or "wait" in norm_cmd:
            self.pause_command_logic()
        elif norm_cmd in ("select", "target") or norm_cmd.startswith("select "):
            now = time.monotonic()
            if now - self._last_select_time < self._select_cooldown_s:
                self.get_logger().info(
                    f"[TARGET] ignoring duplicate select within {self._select_cooldown_s:.1f}s cooldown"
                )
                return
            self._last_select_time = now
            self.select_target_command_logic()
        elif self._is_what_phrase(norm_cmd):
            self.what_are_you_doing_logic()
        elif self._is_confirm_phrase(norm_cmd):
            self.where_are_you_going_logic()
        elif "continue" in norm_cmd or "resume" in norm_cmd:
            self.continue_logic()
        else:
            self.get_logger().warn(f"Unknown voice command: '{raw}'")

    def select_target_command_logic(self) -> None:
        with self._state_lock:
            if self._dialogue_state == DIALOGUE_WAIT_CONFIRM and self._pending_target is not None:
                self.get_logger().info(
                    f"[GATE] ignoring 'select' while waiting for confirm (pending_target={self._pending_target})"
                )
                return
            if self._dialogue_state == DIALOGUE_WAIT_DISPATCH and self._pending_target is not None:
                self.get_logger().info(
                    f"[GATE] ignoring 'select' while pending target is dispatching (pending_target={self._pending_target})"
                )
                return
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
            self._dialogue_state = DIALOGUE_WAIT_CONFIRM
            self._pre_move_gesture_running = True
            state_at_select = self.state
            if state_at_select == "paused":
                self._pre_move_gesture_running = False
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
            self._pre_move_gesture_running = False
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

    def what_are_you_doing_logic(self) -> None:
        with self._state_lock:
            if self._pending_target is None:
                self.get_logger().info(
                    "[GATE] 'what' ignored: no pending target; say 'select' first"
                )
                return
            if self._dialogue_state != DIALOGUE_WAIT_CONFIRM or self._pending_target_confirmed:
                self.get_logger().info(
                    "[GATE] 'what' ignored: only allowed while waiting for confirm"
                )
                return
            if self._pre_move_gesture_running:
                self.get_logger().info("[GESTURE] 'what' ignored: another pre-move gesture is still running")
                return
            gesture_name = self._next_what_gesture
            self._pre_move_gesture_running = True

        try:
            if gesture_name == "wipe":
                ok = self.gesture.wipe_blocking_prefer()
            else:
                ok = self.gesture.grab_blocking_prefer()
        finally:
            with self._state_lock:
                self._pre_move_gesture_running = False

        with self._state_lock:
            if self._pending_target is None:
                self.get_logger().info(
                    f"[GESTURE] pre-move '{gesture_name}' completed but pre-move window already closed"
                )
                return
            if self._dialogue_state != DIALOGUE_WAIT_CONFIRM or self._pending_target_confirmed:
                self.get_logger().info(
                    f"[GESTURE] pre-move '{gesture_name}' completed but dialogue is no longer waiting for confirm"
                )
                return
            if ok:
                self._next_what_gesture = "grab" if gesture_name == "wipe" else "wipe"
                self.get_logger().info(
                    f"[GESTURE] pre-move 'what' gesture executed: {gesture_name} "
                    f"(next={self._next_what_gesture})"
                )
            else:
                self.get_logger().warn(
                    f"[GESTURE] pre-move 'what' gesture failed: {gesture_name} (will retry same next time)"
                )

    def abort_command_logic(self) -> None:
        with self._state_lock:
            self.get_logger().info("[GATE] ABORT requested: stopping all operations")
            if self.state == "aborted":
                self.get_logger().warn("[GATE] robot is already aborted; auto-ready timer already in progress")
                return

            self._resume_state_after_abort = None
            self._suspended_deadline_remaining = None
            self._state_deadline = None
            self._clear_pending_target_after_abort()
            self._set_state("aborted")
            self.gesture.abort()
            self._schedule_abort_auto_ready()
            self.get_logger().info(
                f"[GATE] robot aborted; will auto-return to ready in {self._abort_auto_ready_delay_s:.2f}s"
            )

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
            self.get_logger().info(f"[TARGET] current target: {self._pending_target}")
            if self.current_arm_joint_positions is not None:
                self.get_logger().info(
                    f"[TARGET] current position (joint_1): {self.current_arm_joint_positions[0]:.3f}"
                )
            elif self.current_carriage_pos is not None:
                self.get_logger().info(f"[TARGET] current position: {self.current_carriage_pos:.3f}")
            else:
                self.get_logger().info("[TARGET] current position: unknown (no data yet)")
            if self._pre_move_gesture_running:
                self.get_logger().info(
                    "[TARGET] confirmation deferred: pre-move gesture still running"
                )
                return
            if self._pending_target is None:
                self.get_logger().warn("[TARGET] no pending target to confirm (say 'select' first)")
                return
            if self._dialogue_state != DIALOGUE_WAIT_CONFIRM:
                self.get_logger().info(
                    f"[GATE] confirm ignored: dialogue_state={self._dialogue_state} "
                    f"(pending_target={self._pending_target})"
                )
                return
            self._pending_target_confirmed = True
            self._dialogue_state = DIALOGUE_WAIT_DISPATCH
            self.get_logger().info(
                f"[TARGET] confirmation received for pending target: {self._pending_target}"
            )
            self._publish_pending_target_if_ready()

    def continue_logic(self) -> None:
        reopen_gripper = False
        with self._state_lock:
            self.get_logger().info("[GATE] CONTINUE requested: resuming robot movement")
            if self.state == "paused":
                self._resume_from_pause(auto=False)
                self.get_logger().info("[GATE] robot resumed")
                reopen_gripper = True
            elif self.state == "aborted":
                self._cancel_abort_auto_ready_timer()
                self._clear_pending_target_after_abort()
                next_state = "ready" if self._arm_armed else "initializing"
                self._set_state(next_state)
                if next_state == "ready":
                    self.get_logger().info("[CONTINUE] ready for new target (no active target to resume)")
                else:
                    self.get_logger().info("[CONTINUE] arm not ready yet; staying initializing")
            elif self.state == "ready":
                self.get_logger().info("[CONTINUE] already ready")
            else:
                self.get_logger().warn(f"[GATE] nothing to resume (state={self.state})")

        if reopen_gripper:
            self.gesture.open_no_wait()
            self.get_logger().info("[VOICE] continue -> called /gesture/open to reopen gripper")

    # Main control loop
    def _on_carriage_position(self, msg: Float64) -> None:
        self.current_carriage_pos = msg.data

    def _extract_arm_joint_positions(self, msg: JointState):
        name_to_index = {name: i for i, name in enumerate(msg.name)}
        missing = []
        positions = []
        for joint_name in ARM_JOINT_NAMES:
            idx = name_to_index.get(joint_name)
            if idx is None or idx >= len(msg.position):
                missing.append(joint_name)
                continue
            positions.append(float(msg.position[idx]))
        if missing:
            return None, missing
        return positions, None

    def _on_joint_states_for_arming(self, msg: JointState) -> None:
        arm_positions, missing_joints = self._extract_arm_joint_positions(msg)
        if missing_joints:
            self.current_arm_joint_positions = None
            self._arm_joint_state_valid = False
            signature = (tuple(missing_joints), tuple(msg.name))
            if signature != self._last_joint_missing_signature:
                self._last_joint_missing_signature = signature
                self.get_logger().warn(f"[JOINT] missing={missing_joints} names={list(msg.name)}")
            return

        self.current_arm_joint_positions = arm_positions
        self._last_joint_missing_signature = None
        if not self._arm_joint_state_valid:
            self.get_logger().info(
                f"[JOINT] ok joint_1={arm_positions[0]:.3f} names_len={len(msg.name)}"
            )
        self._arm_joint_state_valid = True

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

    # Helper
    def _select_new_target_safe(self, initial: bool, publish_selected_target: bool = True):
        """Select a new target and optionally publish /selected_target."""
        try:
            prev = gv.currentTargetGlobal
            target = selectNewTarget(None if initial else prev)

            # Force-update global
            gv.currentTargetGlobal = target
            
            self.get_logger().info(f"[TARGET] selected target: {target}")
            if publish_selected_target:
                self._publish_selected_target(target)
            else:
                self.get_logger().info(
                    f"[TARGET] not publishing /selected_target while state={self.state}, "
                    f"dialogue_state={self._dialogue_state}; stored only"
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
                    f"[TARGET] not publishing /selected_target while state={self.state}, "
                    f"dialogue_state={self._dialogue_state}; stored fallback only"
                )
            return gv.currentTargetGlobal

    # Robot init 
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

    def _clear_pending_target_after_abort(self) -> None:
        had_pending = self._pending_target is not None or self._pending_target_confirmed
        self._pending_target = None
        self._pending_target_confirmed = False
        self._pre_move_gesture_running = False
        self._dialogue_state = DIALOGUE_WAIT_SELECT
        gv.currentTargetGlobal = None
        if had_pending:
            self.get_logger().info("[ABORT] cleared pending target/confirmation state")

    def _cancel_abort_auto_ready_timer(self) -> None:
        timer = self._abort_auto_ready_timer
        if timer is None:
            return
        self._abort_auto_ready_timer = None
        try:
            timer.cancel()
        except Exception:
            pass
        try:
            self.destroy_timer(timer)
        except Exception:
            pass

    def _schedule_abort_auto_ready(self) -> None:
        self._cancel_abort_auto_ready_timer()
        self._abort_auto_ready_timer = self.create_timer(
            self._abort_auto_ready_delay_s,
            self._on_abort_auto_ready_timer,
            callback_group=self.cb_group,
        )

    def _on_abort_auto_ready_timer(self) -> None:
        with self._state_lock:
            if self.state == "aborted":
                self._resume_state_after_abort = None
                self._suspended_deadline_remaining = None
                self._state_deadline = None
                next_state = "ready" if self._arm_armed else "initializing"
                self._set_state(next_state)
                if next_state == "ready":
                    self.get_logger().info("[GATE] abort complete -> state=ready (waiting for new select)")
                else:
                    self.get_logger().info(
                        "[GATE] abort complete -> state=initializing (waiting for arm ready + new select)"
                    )
        self._cancel_abort_auto_ready_timer()

    def _publish_pending_target_if_ready(self) -> None:
        if self._pending_target is None:
            self._dialogue_state = DIALOGUE_WAIT_SELECT
            return
        if not self._pending_target_confirmed:
            self._dialogue_state = DIALOGUE_WAIT_CONFIRM
            self.get_logger().info(
                f"[TARGET] pending target retained ({self._pending_target}); waiting for 'where are you going'"
            )
            return
        if self.state != "ready":
            self._dialogue_state = DIALOGUE_WAIT_DISPATCH
            self.get_logger().info(
                f"[TARGET] pending target retained ({self._pending_target}); "
                f"state={self.state}, dialogue_state={self._dialogue_state}"
            )
            return
        target = self._pending_target
        self._pending_target = None
        self._pending_target_confirmed = False
        self._dialogue_state = DIALOGUE_WAIT_SELECT
        self._publish_selected_target(target)
        self.get_logger().info(f"[TARGET] dispatched pending target: {target}")

    def _normalize_voice_text(self, text: str) -> str:
        text = (text or "").lower()
        text = re.sub(r"[^a-z0-9\s]+", " ", text)
        return " ".join(text.split())

    def _is_what_phrase(self, norm_cmd: str) -> bool:
        return norm_cmd == "what" or norm_cmd == "what are you doing" or (
            "what" in norm_cmd
        )

    def _is_confirm_phrase(self, norm_cmd: str) -> bool:
        if norm_cmd in {"where", "confirm"} or "where" in norm_cmd:
            return True
        return norm_cmd in {
            "where are you going",
            "where will you go",
            "where you going",
            "where are we going",
        }


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
