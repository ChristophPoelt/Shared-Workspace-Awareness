#!/usr/bin/env python3
import time
import threading
import os
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
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

WIPE_A = [0.002, -0.8, -1.38, -0.06, -1.0, -0.024952895806755946]
WIPE_B = [0.002, -0.8, -1.38, -0.06, -1.0, -1.55]

GRAB_A = [0.0, -1.1007484589848245, -1.0834244011623186, -3.1319838748875948, 1.6463307101467277, -1.55]
GRAB_B = [0.0, -0.8685842253514816, -1.6931730916470766, -3.0739184520643383, 0.7558960918470252, -1.55]
WIPE_JOINT6_OFFSETS = [0.0, 1.0, -1.0, 0.0, 1.0, -1.0, 0.0]
GRAB_JOINT3_OFFSETS = [0.0, 1.0, -1.0, 0.0, 1.0, -1.0, 0.0]
GESTURE_SEGMENT_S = 1.0
GESTURE_WAIT_MARGIN_S = 0.2
class BackendMode(str, Enum):
    AUTO = "auto"     # action if server exists, else sim topic
    ACTION = "action" # always action (error if server not available)
    SIM = "sim"       # always sim topic


class RobotGestures(Node):
    """
    Gesture node for Robotiq gripper.

    Services (std_srvs/Trigger):
      - /gesture/target_selected : CLOSE -> OPEN -> CLOSE -> OPEN (fire-and-forget)
      - /gesture/target_selected_blocking : same sequence, response returns after completion
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
        self.declare_parameter("follow_control_state", True)

        # Action backend (real robot)
        self.declare_parameter("gripper_action_name", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("open_position", 0.0)
        self.declare_parameter("close_position", 0.8)
        self.declare_parameter("max_effort", 100.0)

        # Arm trajectory publisher (real robot OR sim if controller exists)
        self.declare_parameter("arm_traj_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("arm_move_s", 4.0)   # time for each A->B move
        self.declare_parameter("arm_settle_s", 0.2) # tiny settle between steps
        self.declare_parameter("wipe_joint6_amp", 0.6)
        self.declare_parameter("grab_joint3_amp", 0.25)

        # Sim backend (topic)
        self.declare_parameter("sim_gripper_topic", "/gripper_controller/commands")

        # Gesture timing
        # Longer dwell so sim gripper motion is visible between sequence steps.
        self.declare_parameter("pause_s", 1.0)

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
        self._follow_control_state = bool(self.get_parameter("follow_control_state").value)

        self._arm_traj_topic = str(self.get_parameter("arm_traj_topic").value)
        self._arm_move_s = float(self.get_parameter("arm_move_s").value)
        self._arm_settle_s = float(self.get_parameter("arm_settle_s").value)
        self._wipe_joint6_amp = float(self.get_parameter("wipe_joint6_amp").value)
        self._grab_joint3_amp = float(self.get_parameter("grab_joint3_amp").value)

        # --------------------------
        # State
        # --------------------------
        self._lock = threading.Lock()
        self._busy: bool = False
        self._paused: bool = False
        self._abort_requested: bool = False

        self._last_goal_handle: Optional[ClientGoalHandle] = None
        self._latest_joint_state: Optional[JointState] = None
        self._joint_state_cv = threading.Condition()

        # --------------------------
        # Gripper interfaces
        # --------------------------
        self._gripper_client = ActionClient(self, GripperCommand, self._action_name)

        # Match gripper controller subscriber QoS (RELIABLE + VOLATILE).
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self._sim_pub = self.create_publisher(Float64MultiArray, self._sim_topic, qos)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._state_sub = None
        if self._follow_control_state:
            self._state_sub = self.create_subscription(
                String, "/robot_control_state", self._on_control_state, qos
            )

        self._arm_pub = self.create_publisher(JointTrajectory, self._arm_traj_topic, qos)
        self.get_logger().info(f"[GESTURE] arm traj topic: {self._arm_traj_topic} (move_s={self._arm_move_s})")

        # --------------------------
        # Services
        # --------------------------
        self.create_service(Trigger, "/gesture/target_selected", self._on_target_selected)
        self.create_service(
            Trigger, "/gesture/target_selected_blocking", self._on_target_selected_blocking
        )
        self.create_service(Trigger, "/gesture/pause", self._on_pause)
        self.create_service(Trigger, "/gesture/resume", self._on_resume)
        self.create_service(Trigger, "/gesture/abort", self._on_abort)
        self.create_timer(5.0, self._warn_on_duplicate_node_names)

        self.create_service(Trigger, "/gesture/wipe", self._on_wipe)
        self.create_service(Trigger, "/gesture/wipe_blocking", self._on_wipe_blocking)
        self.create_service(Trigger, "/gesture/grab", self._on_grab)
        self.create_service(Trigger, "/gesture/grab_blocking", self._on_grab_blocking)

        # --------------------------
        # Log startup
        # --------------------------
        self.get_logger().info(
            f"[INIT] node={self.get_name()} pid={os.getpid()} RobotGestures ready"
        )
        self.get_logger().info(f"[GESTURE] backend mode: {self._backend_mode.value}")
        self.get_logger().info(
            f"[GESTURE] action={self._action_name} (open={self._open_pos}, close={self._close_pos}, "
            f"effort={self._max_effort})"
        )
        self.get_logger().info(f"[GESTURE] sim topic: {self._sim_topic}")
        self.get_logger().info(
            "[GATE] robot_gestures does not publish /robot_control_state or /arm_armed; "
            "it consumes services and optionally follows /robot_control_state"
        )

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

    def _warn_on_duplicate_node_names(self) -> None:
        try:
            names = [name for name, _ns in self.get_node_names_and_namespaces()]
        except Exception:
            return
        duplicates = sum(1 for name in names if name == self.get_name())
        if duplicates > 1:
            self.get_logger().warn(
                f"[INIT] duplicate node name detected: {self.get_name()} count={duplicates}. "
                "Stop duplicate launch/CLI instances to avoid conflicting gesture behavior."
            )

    def _on_control_state(self, msg: String) -> None:
        state = (msg.data or "").strip().lower()
        if not state:
            return
        if state == "paused":
            self.get_logger().info("[GATE] /robot_control_state=paused -> pausing gesture execution")
            with self._lock:
                self._paused = True
            return
        if state == "aborted":
            self.get_logger().info("[GATE] /robot_control_state=aborted -> aborting gesture execution")
            with self._lock:
                self._abort_requested = True
                self._paused = False
            self._abort_best_effort()
            return
        if state == "ready":
            with self._lock:
                if self._paused:
                    self.get_logger().info("[GATE] /robot_control_state=ready -> resuming gesture execution")
                self._paused = False

    def _on_joint_state(self, msg: JointState) -> None:
        with self._joint_state_cv:
            self._latest_joint_state = msg
            self._joint_state_cv.notify_all()

    def _wait_for_joint_state(self, timeout_s: float = 0.5) -> Optional[JointState]:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._joint_state_cv:
            while self._latest_joint_state is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._joint_state_cv.wait(timeout=remaining)
            return self._latest_joint_state

    # --------------------------
    # Services
    # --------------------------
    def _begin_target_selected_sequence(self):
        with self._lock:
            if self._busy:
                return False, "Busy: another gesture is running."
            self._busy = True
            self._paused = False
            self._abort_requested = False
        return True, "Started: target_selected gesture."

    def _on_target_selected(self, request, response):
        self.get_logger().info("[GESTURE] service hit: /gesture/target_selected")
        ok, msg = self._begin_target_selected_sequence()
        if not ok:
            response.success = False
            response.message = msg
            return response

        threading.Thread(target=self._run_target_selected_sequence, daemon=True).start()

        response.success = True
        response.message = msg
        return response

    def _on_target_selected_blocking(self, request, response):
        self.get_logger().info("[GESTURE] service hit: /gesture/target_selected_blocking")
        ok, msg = self._begin_target_selected_sequence()
        if not ok:
            response.success = False
            response.message = msg
            return response
        ok, msg = self._run_target_selected_sequence()
        response.success = ok
        response.message = msg
        return response

    def _on_pause(self, request, response):
        self.get_logger().info("[GESTURE] service hit: /gesture/pause")
        with self._lock:
            self._paused = True

        ok, msg = self._close_gripper_best_effort()
        response.success = ok
        response.message = msg
        return response

    def _on_resume(self, request, response):
        self.get_logger().info("[GESTURE] service hit: /gesture/resume")
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
        self.get_logger().info("[GESTURE] service hit: /gesture/abort")
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
            self.get_logger().info("[GESTURE] sequence: CLOSE -> OPEN -> CLOSE -> OPEN")

            seq = [self._close_pos, self._open_pos, self._close_pos, self._open_pos]

            for i, pos in enumerate(seq):
                self._check_abort_or_pause()
                self._send_gripper_no_wait(pos)

                # no extra sleep after last command
                if i < len(seq) - 1:
                    self._sleep_with_checks(self._pause_s)

            self.get_logger().info("[RESULT] target_selected sequence done (ends OPEN)")
            return True, "Completed: target_selected gesture."

        except RuntimeError as e:
            self.get_logger().warn(f"[RESULT] gesture stopped: {e}")
            return False, f"Stopped: {e}"
        except Exception as e:
            self.get_logger().error(f"[RESULT] gesture failed: {e}")
            return False, f"Failed: {e}"
        finally:
            with self._lock:
                self._busy = False
                self._paused = False
                self._abort_requested = False

    ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    def _begin_sequence(self, name: str):
        with self._lock:
            if self._busy:
                return False, f"Busy: another gesture is running."
            self._busy = True
            self._paused = False
            self._abort_requested = False
        return True, f"Started: {name} gesture."

    def _publish_arm_target(self, positions, move_s: float):
        self._publish_arm_target_named(self.ARM_JOINT_NAMES, positions, move_s)

    def _publish_arm_target_named(self, joint_names, positions, move_s: float):
        self._publish_arm_trajectory_named(joint_names, [positions], [float(move_s)])

    def _publish_arm_trajectory_named(self, joint_names, waypoints, time_from_starts_s):
        if len(waypoints) != len(time_from_starts_s):
            raise RuntimeError(
                f"Waypoint/time length mismatch: {len(waypoints)} waypoints vs {len(time_from_starts_s)} times"
            )

        traj = JointTrajectory()
        traj.joint_names = [str(j) for j in joint_names]

        traj.points = []
        for positions, move_s in zip(waypoints, time_from_starts_s):
            if len(joint_names) != len(positions):
                raise RuntimeError(
                    f"Joint name/position length mismatch: {len(joint_names)} names vs {len(positions)} positions"
                )
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in positions]
            sec = int(move_s)
            nsec = int((move_s - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(pt)
        self._arm_pub.publish(traj)

        subs = self._arm_pub.get_subscription_count()
        self.get_logger().info(
            f"[GESTURE] arm trajectory {len(traj.points)} points joints={traj.joint_names} "
            f"(T_end={float(time_from_starts_s[-1]):.2f}s) on {self._arm_traj_topic} (subs={subs})"
        )

    def _single_joint_gesture_targets(self, joint_name: str, offsets_rad, fallback_joint_name: Optional[str] = None):
        joint_state = self._wait_for_joint_state(timeout_s=0.5)
        if joint_state is None:
            raise RuntimeError("no /joint_states available for gesture baseline")

        name_to_position = dict(zip(joint_state.name, joint_state.position))
        chosen_joint = None
        if joint_name in self.ARM_JOINT_NAMES and joint_name in name_to_position:
            chosen_joint = joint_name
        elif (
            fallback_joint_name
            and fallback_joint_name in self.ARM_JOINT_NAMES
            and fallback_joint_name in name_to_position
        ):
            chosen_joint = fallback_joint_name
        else:
            details = f"{joint_name}" + (f" (fallback {fallback_joint_name})" if fallback_joint_name else "")
            raise RuntimeError(f"gesture joint missing in /joint_states or controller list: {details}")

        missing = [j for j in self.ARM_JOINT_NAMES if j not in name_to_position]
        if missing:
            raise RuntimeError(f"missing joints in /joint_states for full-arm gesture: {missing}")

        baseline_positions = [float(name_to_position[j]) for j in self.ARM_JOINT_NAMES]
        joint_idx = self.ARM_JOINT_NAMES.index(chosen_joint)
        baseline = baseline_positions[joint_idx]
        target_waypoints = [baseline + float(offset) for offset in offsets_rad]

        targets = []
        for value in target_waypoints:
            waypoint = list(baseline_positions)
            waypoint[joint_idx] = float(value)
            targets.append(waypoint)
        return list(self.ARM_JOINT_NAMES), chosen_joint, joint_idx, baseline, targets

    def _run_joint_pattern(self, name: str, A, B):
        """
        Runs A -> B -> A -> B with pause/abort checks.
        """
        try:
            self.get_logger().info(f"[GESTURE] sequence {name}: A -> B -> A -> B")

            seq = [A, B, A, B]
            for i, target in enumerate(seq):
                self._check_abort_or_pause()
                self._publish_arm_target(target, self._arm_move_s)

                # wait for motion to (mostly) complete
                self._sleep_with_checks(self._arm_move_s)

                # small settle, not after last step
                if i < len(seq) - 1 and self._arm_settle_s > 0.0:
                    self._sleep_with_checks(self._arm_settle_s)

            self.get_logger().info(f"[RESULT] {name} sequence done")
            return True, f"Completed: {name} gesture."

        except RuntimeError as e:
            self.get_logger().warn(f"[RESULT] {name} stopped: {e}")
            return False, f"Stopped: {e}"
        except Exception as e:
            self.get_logger().error(f"[RESULT] {name} failed: {e}")
            return False, f"Failed: {e}"
        finally:
            with self._lock:
                self._busy = False
                self._paused = False
                self._abort_requested = False

    def _run_single_joint_relative_pattern(
        self,
        name: str,
        offsets_rad,
        *,
        joint_name: str,
        amplitude_rad: float,
        fallback_joint_name: Optional[str] = None,
    ):
        try:
            self.get_logger().info(f"[GESTURE] sequence {name}: single-joint relative pattern")
            scaled_offsets = [float(amplitude_rad) * float(v) for v in offsets_rad]
            joint_names, used_joint_name, used_joint_idx, baseline, targets = self._single_joint_gesture_targets(
                joint_name,
                scaled_offsets,
                fallback_joint_name=fallback_joint_name,
            )
            times = [float(i) * GESTURE_SEGMENT_S for i in range(len(targets))]
            waypoints = [float(target[used_joint_idx]) for target in targets]
            total_duration_s = float(times[-1]) if times else 0.0

            self.get_logger().info(
                f"[GESTURE] {name} joint trajectory joint={used_joint_name} baseline={baseline:.4f} "
                f"waypoints={[round(v, 4) for v in waypoints]} "
                f"total_duration={total_duration_s:.2f}s"
            )
            if name == "grab":
                self.get_logger().info(f"[GESTURE] grab using joint={used_joint_name}")

            self._check_abort_or_pause()
            self._publish_arm_trajectory_named(joint_names, targets, times)
            self._sleep_with_checks(total_duration_s + GESTURE_WAIT_MARGIN_S)

            self.get_logger().info(f"[RESULT] {name} sequence done")
            return True, f"Completed: {name} gesture."
        except RuntimeError as e:
            self.get_logger().warn(f"[RESULT] {name} stopped: {e}")
            return False, f"Stopped: {e}"
        except Exception as e:
            self.get_logger().error(f"[RESULT] {name} failed: {e}")
            return False, f"Failed: {e}"
        finally:
            with self._lock:
                self._busy = False
                self._paused = False
                self._abort_requested = False

    # --------------------------
    # Wipe services
    # --------------------------
    def _on_wipe(self, request, response):
        self.get_logger().info("[GESTURE] service hit: /gesture/wipe")
        ok, msg = self._begin_sequence("wipe")
        if not ok:
            response.success = False
            response.message = msg
            return response

        threading.Thread(
            target=lambda: self._run_single_joint_relative_pattern(
                "wipe",
                WIPE_JOINT6_OFFSETS,
                joint_name="joint_6",
                amplitude_rad=self._wipe_joint6_amp,
            ),
            daemon=True
        ).start()

        response.success = True
        response.message = msg
        return response

    def _on_wipe_blocking(self, request, response):
        self.get_logger().info("[GESTURE] service hit: /gesture/wipe_blocking")
        ok, msg = self._begin_sequence("wipe")
        if not ok:
            response.success = False
            response.message = msg
            return response

        ok2, msg2 = self._run_single_joint_relative_pattern(
            "wipe",
            WIPE_JOINT6_OFFSETS,
            joint_name="joint_6",
            amplitude_rad=self._wipe_joint6_amp,
        )
        response.success = ok2
        response.message = msg2
        return response

    # --------------------------
    # Grab services
    # --------------------------
    def _on_grab(self, request, response):
        self.get_logger().info("[GESTURE] service hit: /gesture/grab")
        ok, msg = self._begin_sequence("grab")
        if not ok:
            response.success = False
            response.message = msg
            return response

        threading.Thread(
            target=lambda: self._run_single_joint_relative_pattern(
                "grab",
                GRAB_JOINT3_OFFSETS,
                joint_name="joint_3",
                fallback_joint_name="joint_2",
                amplitude_rad=self._grab_joint3_amp,
            ),
            daemon=True
        ).start()

        response.success = True
        response.message = msg
        return response

    def _on_grab_blocking(self, request, response):
        self.get_logger().info("[GESTURE] service hit: /gesture/grab_blocking")
        ok, msg = self._begin_sequence("grab")
        if not ok:
            response.success = False
            response.message = msg
            return response

        ok2, msg2 = self._run_single_joint_relative_pattern(
            "grab",
            GRAB_JOINT3_OFFSETS,
            joint_name="joint_3",
            fallback_joint_name="joint_2",
            amplitude_rad=self._grab_joint3_amp,
        )
        response.success = ok2
        response.message = msg2
        return response

    # --------------------------
    # Sending gripper commands
    # --------------------------
    def _send_gripper_no_wait(self, position: float) -> None:
        backend = self._select_backend()
        self.get_logger().info(f"[GESTURE] backend={backend.value}, pos={position}")

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
        self.get_logger().info(f"[GESTURE] sim pub {msg.data[0]} -> {self._sim_topic} (subs={subs})")

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
            self.get_logger().warn("[GESTURE] abort: canceling last gripper goal (best-effort)")
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
