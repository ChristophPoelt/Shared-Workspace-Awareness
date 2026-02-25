#!/usr/bin/env python3
import copy
import math
import os
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from .angle_calculation import ClockwiseRail

# Numeric positions for targets
TARGET_POSITIONS = {
    "position0": 2.7,
    "position1": -2.5,
    "position2": -1.0,
}

ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
FOLDED = [0.002, -0.8, -1.38, -0.06, -1.0, -1.55]  # full 6 joints, joint_1 overridden
STRETCHED = [0.0, 0.5, -0.30, -0.0, -0.5, -1.55]  # full 6 joints, joint_1 overridden


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def shortest_angle_diff(target: float, current: float) -> float:
    return (target - current + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class MoveItExecutionOutcome:
    request_id: int
    context_id: int
    phase: str
    status: str  # succeeded|failed|canceled
    action_status: int
    error_code: int
    error_name: str
    detail: str


class MoveItArmBackend:
    def __init__(
        self,
        node: Node,
        *,
        group_name: str,
        action_name: str,
        server_wait_s: float,
        allowed_planning_time_s: float,
        num_planning_attempts: int,
        velocity_scaling: float,
        acceleration_scaling: float,
        joint_goal_tolerance_rad: float,
        planner_id: str = "",
        pipeline_id: str = "",
        replan: bool = False,
        replan_attempts: int = 0,
        replan_delay_s: float = 0.0,
    ):
        self._node = node
        self._log = node.get_logger()
        self._group_name = group_name
        self._action_name = action_name
        self._server_wait_s = max(0.0, float(server_wait_s))
        self._allowed_planning_time_s = max(0.1, float(allowed_planning_time_s))
        self._num_planning_attempts = max(1, int(num_planning_attempts))
        self._velocity_scaling = self._sanitize_scaling(velocity_scaling)
        self._acceleration_scaling = self._sanitize_scaling(acceleration_scaling)
        self._joint_goal_tolerance_rad = max(1e-4, float(joint_goal_tolerance_rad))
        self._planner_id = str(planner_id)
        self._pipeline_id = str(pipeline_id)
        self._replan = bool(replan)
        self._replan_attempts = max(0, int(replan_attempts))
        self._replan_delay_s = max(0.0, float(replan_delay_s))

        self._client = ActionClient(node, MoveGroup, self._action_name)
        self._request_seq = 0
        self._active_request_id: Optional[int] = None
        self._active_context_id: int = 0
        self._active_phase: str = ""
        self._active_goal_handle: Optional[ClientGoalHandle] = None
        self._busy = False
        self._cancel_requested = False
        self._latest_feedback_state: Optional[str] = None
        self._latest_outcome: Optional[MoveItExecutionOutcome] = None

        self._error_name_by_code = {}
        for name in dir(MoveItErrorCodes):
            if not name.isupper():
                continue
            value = getattr(MoveItErrorCodes, name)
            if isinstance(value, int):
                self._error_name_by_code[value] = name

    @staticmethod
    def _sanitize_scaling(value: float) -> float:
        value = float(value)
        if value <= 0.0:
            return 0.2
        return clamp(value, 1e-3, 1.0)

    def is_busy(self) -> bool:
        return self._busy

    def active_request_id(self) -> Optional[int]:
        return self._active_request_id

    def consume_outcome(self) -> Optional[MoveItExecutionOutcome]:
        outcome = self._latest_outcome
        self._latest_outcome = None
        return outcome

    def stop(self, reason: str) -> None:
        if not self._busy and self._active_goal_handle is None:
            return

        self._cancel_requested = True
        rid = self._active_request_id
        phase = self._active_phase or "unknown"
        self._log.info(f"[MOVEIT] stop requested (phase={phase}, request_id={rid}, reason={reason})")

        if self._active_goal_handle is not None:
            cancel_future = self._active_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda fut, request_id=rid: self._on_cancel_response(fut, request_id)
            )

    def send_joint_goal(
        self,
        *,
        phase: str,
        context_id: int,
        desired_positions,
        start_joint_state: JointState,
        stretch_joint1_path_constraint: Optional[tuple] = None,
    ) -> bool:
        if self._busy:
            return False

        if not self._client.wait_for_server(timeout_sec=self._server_wait_s):
            self._log.warn(
                f"[MOVEIT] action server not available on {self._action_name}; "
                f"phase={phase} dispatch skipped"
            )
            return False

        goal_msg = MoveGroup.Goal()
        req = goal_msg.request
        req.group_name = self._group_name
        req.num_planning_attempts = self._num_planning_attempts
        req.allowed_planning_time = self._allowed_planning_time_s
        req.max_velocity_scaling_factor = self._velocity_scaling
        req.max_acceleration_scaling_factor = self._acceleration_scaling
        req.planner_id = self._planner_id
        req.pipeline_id = self._pipeline_id
        req.start_state.is_diff = False
        req.start_state.joint_state = copy.deepcopy(start_joint_state)
        req.goal_constraints = [self._build_goal_constraints(desired_positions)]

        if stretch_joint1_path_constraint is not None:
            joint_name, joint_pos, joint_tol = stretch_joint1_path_constraint
            path_constraints = Constraints()
            jc = JointConstraint()
            jc.joint_name = str(joint_name)
            jc.position = float(joint_pos)
            jc.tolerance_above = float(joint_tol)
            jc.tolerance_below = float(joint_tol)
            jc.weight = 1.0
            path_constraints.joint_constraints = [jc]
            req.path_constraints = path_constraints

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.replan = self._replan
        goal_msg.planning_options.replan_attempts = self._replan_attempts
        goal_msg.planning_options.replan_delay = self._replan_delay_s
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

        self._request_seq += 1
        request_id = self._request_seq
        self._active_request_id = request_id
        self._active_context_id = int(context_id)
        self._active_phase = str(phase)
        self._active_goal_handle = None
        self._busy = True
        self._cancel_requested = False
        self._latest_feedback_state = None

        self._log.info(
            f"[MOVEIT] plan+execute dispatch phase={phase} request_id={request_id} "
            f"context={context_id} group={self._group_name} action={self._action_name}"
        )

        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=lambda msg, rid=request_id: self._on_feedback(msg, rid),
        )
        send_future.add_done_callback(
            lambda fut, rid=request_id: self._on_goal_response(fut, rid)
        )
        return True

    def _build_goal_constraints(self, desired_positions) -> Constraints:
        constraints = Constraints()
        joint_constraints = []
        for joint_name, position in zip(ARM_JOINT_NAMES, desired_positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(position)
            jc.tolerance_above = self._joint_goal_tolerance_rad
            jc.tolerance_below = self._joint_goal_tolerance_rad
            jc.weight = 1.0
            joint_constraints.append(jc)
        constraints.joint_constraints = joint_constraints
        return constraints

    def _on_feedback(self, feedback_msg, request_id: int) -> None:
        if request_id != self._active_request_id:
            return
        try:
            state = str(feedback_msg.feedback.state)
        except Exception:
            return
        if state != self._latest_feedback_state:
            self._latest_feedback_state = state
            self._log.info(
                f"[MOVEIT] feedback phase={self._active_phase} request_id={request_id} state={state}"
            )

    def _on_goal_response(self, future, request_id: int) -> None:
        if request_id != self._active_request_id:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._finalize_outcome(
                request_id=request_id,
                status="failed",
                action_status=GoalStatus.STATUS_ABORTED,
                error_code=MoveItErrorCodes.FAILURE,
                detail=f"send_goal_async failed: {exc}",
            )
            return

        if goal_handle is None or not goal_handle.accepted:
            self._finalize_outcome(
                request_id=request_id,
                status="failed",
                action_status=GoalStatus.STATUS_ABORTED,
                error_code=MoveItErrorCodes.FAILURE,
                detail="goal rejected by move_group",
            )
            return

        self._active_goal_handle = goal_handle
        self._log.info(
            f"[MOVEIT] goal accepted phase={self._active_phase} request_id={request_id}"
        )

        if self._cancel_requested:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                lambda fut, rid=request_id: self._on_cancel_response(fut, rid)
            )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut, rid=request_id: self._on_result(fut, rid)
        )

    def _on_cancel_response(self, future, request_id: Optional[int]) -> None:
        if request_id is not None and request_id != self._active_request_id:
            return
        try:
            response = future.result()
            num_canceling = len(getattr(response, "goals_canceling", []))
        except Exception as exc:
            self._log.warn(f"[MOVEIT] cancel request error: {exc}")
            return
        self._log.info(
            f"[MOVEIT] cancel requested acknowledged (request_id={request_id}, "
            f"goals_canceling={num_canceling})"
        )

    def _on_result(self, future, request_id: int) -> None:
        if request_id != self._active_request_id:
            return

        try:
            wrapped_result = future.result()
        except Exception as exc:
            self._finalize_outcome(
                request_id=request_id,
                status="failed",
                action_status=GoalStatus.STATUS_ABORTED,
                error_code=MoveItErrorCodes.FAILURE,
                detail=f"result future failed: {exc}",
            )
            return

        action_status = int(getattr(wrapped_result, "status", GoalStatus.STATUS_UNKNOWN))
        result_msg = getattr(wrapped_result, "result", None)
        error_code = MoveItErrorCodes.FAILURE
        detail = ""
        if result_msg is not None and hasattr(result_msg, "error_code"):
            error = result_msg.error_code
            error_code = int(getattr(error, "val", MoveItErrorCodes.FAILURE))
            error_msg = str(getattr(error, "message", "") or "").strip()
            error_src = str(getattr(error, "source", "") or "").strip()
            if error_msg or error_src:
                detail = f"{error_src}: {error_msg}".strip(": ")

        if action_status == GoalStatus.STATUS_CANCELED or error_code == MoveItErrorCodes.PREEMPTED:
            status = "canceled"
        elif action_status == GoalStatus.STATUS_SUCCEEDED and error_code == MoveItErrorCodes.SUCCESS:
            status = "succeeded"
        else:
            status = "failed"

        self._finalize_outcome(
            request_id=request_id,
            status=status,
            action_status=action_status,
            error_code=error_code,
            detail=detail,
        )

    def _finalize_outcome(
        self,
        *,
        request_id: int,
        status: str,
        action_status: int,
        error_code: int,
        detail: str,
    ) -> None:
        if request_id != self._active_request_id:
            return

        error_name = self._error_name_by_code.get(int(error_code), f"UNKNOWN({error_code})")
        phase = self._active_phase
        context_id = self._active_context_id
        self._busy = False
        self._active_goal_handle = None
        self._cancel_requested = False
        self._latest_feedback_state = None

        if status == "succeeded":
            self._log.info(
                f"[MOVEIT] success phase={phase} request_id={request_id} "
                f"action_status={action_status} error={error_name} error_code={int(error_code)}"
            )
        elif status == "canceled":
            self._log.info(
                f"[MOVEIT] canceled phase={phase} request_id={request_id} "
                f"action_status={action_status} error={error_name} error_code={int(error_code)}"
                + (f" detail={detail}" if detail else "")
            )
        else:
            self._log.warn(
                f"[MOVEIT] failure phase={phase} request_id={request_id} "
                f"action_status={action_status} error={error_name} error_code={int(error_code)}"
                + (f" detail={detail}" if detail else "")
            )

        self._latest_outcome = MoveItExecutionOutcome(
            request_id=request_id,
            context_id=context_id,
            phase=phase,
            status=status,
            action_status=action_status,
            error_code=int(error_code),
            error_name=error_name,
            detail=detail,
        )


class PointJoint1Node(Node):
    def __init__(self):
        super().__init__("point_joint1_node_logic")

        self.declare_parameter("command_period_s", 0.2)
        self.declare_parameter("joint1_deadband_rad", 0.01)
        self.declare_parameter("yaw_deadband_rad", 0.03)
        self.declare_parameter("fold_tolerance_rad", 0.05)
        self.declare_parameter("replan_cooldown_s", 0.4)
        self.declare_parameter("fixed_rail_pos", 0.0)
        self.declare_parameter("folded_pose", FOLDED)
        self.declare_parameter("stretched_pose", STRETCHED)

        self.declare_parameter("moveit_action_name", "/move_action")
        self.declare_parameter("moveit_group_name", "manipulator")
        self.declare_parameter("moveit_server_wait_s", 0.2)
        self.declare_parameter("moveit_allowed_planning_time_s", 2.0)
        self.declare_parameter("moveit_num_planning_attempts", 3)
        self.declare_parameter("moveit_velocity_scaling", 0.2)
        self.declare_parameter("moveit_acceleration_scaling", 0.2)
        self.declare_parameter("moveit_joint_goal_tolerance_rad", 0.02)
        self.declare_parameter("moveit_planner_id", "")
        self.declare_parameter("moveit_pipeline_id", "")
        self.declare_parameter("moveit_replan", False)
        self.declare_parameter("moveit_replan_attempts", 0)
        self.declare_parameter("moveit_replan_delay_s", 0.0)
        self.declare_parameter("constrain_joint1_during_stretching", True)
        self.declare_parameter("stretch_joint1_path_tolerance_rad", 0.10)
        self.declare_parameter("demo_auto_ready", False)
        self.declare_parameter("demo_auto_armed", False)
        self.declare_parameter("demo_override_grace_s", 1.0)

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # Command/gating topics should be easy to publish from CLI; no latching needed.
            durability=DurabilityPolicy.VOLATILE,
        )
        target_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            # Target selections are command events, so use VOLATILE instead of TRANSIENT_LOCAL.
            durability=DurabilityPolicy.VOLATILE,
        )

        self.command_period_s = self._positive_param("command_period_s", 0.2)
        self.joint1_deadband_rad = self._nonnegative_param("joint1_deadband_rad", 0.01)
        self.yaw_deadband_rad = self._nonnegative_param("yaw_deadband_rad", 0.03)
        self.fold_tolerance_rad = self._nonnegative_param("fold_tolerance_rad", 0.05)
        self.replan_cooldown_s = self._nonnegative_param("replan_cooldown_s", 0.4)
        self.fixed_rail_pos = float(self.get_parameter("fixed_rail_pos").value)
        self.folded_pose = self._pose_param("folded_pose", FOLDED)
        self.stretched_pose = self._pose_param("stretched_pose", STRETCHED)
        self.constrain_joint1_during_stretching = bool(
            self.get_parameter("constrain_joint1_during_stretching").value
        )
        self.stretch_joint1_path_tolerance_rad = self._nonnegative_param(
            "stretch_joint1_path_tolerance_rad", 0.10
        )

        self.current_target = None
        self._target_sub = self.create_subscription(String, "/selected_target", self._on_target, target_qos)
        self.control_state = "initializing"
        self._control_state_sub = self.create_subscription(
            String, "/robot_control_state", self._on_control_state, state_qos
        )
        self.arm_armed = False
        self._arm_armed_sub = self.create_subscription(Bool, "/arm_armed", self._on_arm_armed, state_qos)

        self.current_joint_state = None
        self.current_rail_pos = self.fixed_rail_pos
        self.has_carriage_feedback = False
        self._missing_joints_warned = False
        self._last_sent_yaw = None

        self.phase = "idle"  # idle|folding|turning|stretching
        self._sequence_generation = 0
        self._cooldown_until = 0.0

        # Subscribe to joint states
        self.create_subscription(JointState, "/joint_states", self.joint_state_cb, 10)

        # Subscribe to carriage position
        self.create_subscription(Float64, "/elmo/id1/carriage/position/get", self.carriage_pos_cb, 10)

        self.moveit_group_name = str(self.get_parameter("moveit_group_name").value).strip() or "manipulator"
        if self.moveit_group_name == "arm":
            self.get_logger().warn(
                "[MOVEIT] moveit_group_name='arm' is invalid for this config; overriding to 'manipulator'"
            )
            self.moveit_group_name = "manipulator"
        self.moveit_action_name = str(self.get_parameter("moveit_action_name").value).strip() or "/move_action"
        self.demo_auto_ready = bool(self.get_parameter("demo_auto_ready").value)
        self.demo_auto_armed = bool(self.get_parameter("demo_auto_armed").value)
        self.demo_override_grace_s = self._nonnegative_param("demo_override_grace_s", 1.0)
        self._started_at_s = self.get_clock().now().nanoseconds * 1e-9
        self._demo_override_logged = set()
        self._last_blocked_reason = None
        self._last_blocked_log_s = 0.0
        self._blocked_log_repeat_s = 10.0
        self._gesture_target_selected_cli = self.create_client(Trigger, "/gesture/target_selected")

        self.arm_backend = MoveItArmBackend(
            self,
            group_name=self.moveit_group_name,
            action_name=self.moveit_action_name,
            server_wait_s=self._nonnegative_param("moveit_server_wait_s", 0.2),
            allowed_planning_time_s=self._positive_param("moveit_allowed_planning_time_s", 2.0),
            num_planning_attempts=int(self.get_parameter("moveit_num_planning_attempts").value),
            velocity_scaling=self._positive_param("moveit_velocity_scaling", 0.2),
            acceleration_scaling=self._positive_param("moveit_acceleration_scaling", 0.2),
            joint_goal_tolerance_rad=self._nonnegative_param(
                "moveit_joint_goal_tolerance_rad", 0.02
            ),
            planner_id=str(self.get_parameter("moveit_planner_id").value),
            pipeline_id=str(self.get_parameter("moveit_pipeline_id").value),
            replan=bool(self.get_parameter("moveit_replan").value),
            replan_attempts=int(self.get_parameter("moveit_replan_attempts").value),
            replan_delay_s=self._nonnegative_param("moveit_replan_delay_s", 0.0),
        )

        # Rail math
        self.rail = ClockwiseRail(
            length_long=3.43,
            length_short=0.992,
            radius=0.279,
            zero_offset=0.8,
        )

        self.create_timer(self.command_period_s, self.update_joint1)
        self.create_timer(5.0, self._warn_on_duplicate_node_names)
        self.get_logger().info(
            f"[INIT] node={self.get_name()} pid={os.getpid()} "
            f"Pointing mode (MoveIt backend) enabled with fixed carriage fallback: {self.fixed_rail_pos:.3f}"
        )
        self.get_logger().info(
            "MoveIt pointing timing: "
            f"period={self.command_period_s:.2f}s, "
            f"replan_cooldown={self.replan_cooldown_s:.2f}s, "
            f"yaw_deadband={self.yaw_deadband_rad:.4f} rad, "
            f"fold_tol={self.fold_tolerance_rad:.4f} rad"
        )
        self.get_logger().info(
            f"[MOVEIT] startup group={self.moveit_group_name} action={self.moveit_action_name} "
            f"(joint goal constraints use {ARM_JOINT_NAMES})"
        )
        if self.demo_auto_ready or self.demo_auto_armed:
            self.get_logger().warn(
                "[GATE] demo override enabled "
                f"(demo_auto_ready={self.demo_auto_ready}, demo_auto_armed={self.demo_auto_armed}, "
                f"grace={self.demo_override_grace_s:.1f}s)"
            )

    def _positive_param(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            self.get_logger().warn(f"Parameter {name} must be > 0. Using default {default}.")
            return default
        return value

    def _nonnegative_param(self, name: str, default: float) -> float:
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            self.get_logger().warn(f"Parameter {name} must be >= 0. Using default {default}.")
            return default
        return value

    def _pose_param(self, name: str, default):
        value = self.get_parameter(name).value
        try:
            pose = [float(v) for v in value]
        except Exception:
            self.get_logger().warn(f"Parameter {name} must be a list of 6 floats. Using default.")
            return list(default)
        if len(pose) != 6:
            self.get_logger().warn(f"Parameter {name} must have length 6. Using default.")
            return list(default)
        return pose

    def _set_phase(self, new_phase: str, reason: str = "") -> None:
        if new_phase == self.phase:
            return
        old_phase = self.phase
        self.phase = new_phase
        if reason:
            self.get_logger().info(f"[PHASE] {old_phase} -> {new_phase} ({reason})")
        else:
            self.get_logger().info(f"[PHASE] {old_phase} -> {new_phase}")

    def _bump_sequence_generation(self, reason: str) -> None:
        self._sequence_generation += 1
        self.get_logger().info(
            f"[SEQUENCE] generation={self._sequence_generation} ({reason})"
        )

    def _warn_on_duplicate_node_names(self) -> None:
        try:
            names = [name for name, _ns in self.get_node_names_and_namespaces()]
        except Exception:
            return
        duplicates = sum(1 for name in names if name == self.get_name())
        if duplicates > 1:
            self.get_logger().warn(
                f"[INIT] duplicate node name detected: {self.get_name()} count={duplicates}. "
                "If using wrappers/launch + standalone CLI, ensure only one instance is running."
            )

    def _call_trigger_async(self, client, label: str) -> None:
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(f"[GESTURE] service not available: {label}")
                return
        future = client.call_async(Trigger.Request())

        def _done_cb(fut):
            try:
                resp = fut.result()
                if resp and resp.success:
                    self.get_logger().info(f"[GESTURE] {label}: OK ({resp.message})")
                elif resp:
                    self.get_logger().warn(f"[GESTURE] {label}: FAIL ({resp.message})")
                else:
                    self.get_logger().warn(f"[GESTURE] {label}: no response")
            except Exception as exc:
                self.get_logger().error(f"[GESTURE] {label}: exception: {exc}")

        future.add_done_callback(_done_cb)

    def _trigger_post_stretch_gesture(self, reason: str) -> None:
        self.get_logger().info(f"[GESTURE] triggering post-stretch target_selected ({reason})")
        self._call_trigger_async(self._gesture_target_selected_cli, "/gesture/target_selected")

    def _request_stop(self, reason: str) -> None:
        self.arm_backend.stop(reason)
        self._last_sent_yaw = None
        now = self.get_clock().now().nanoseconds * 1e-9
        self._cooldown_until = max(self._cooldown_until, now + min(self.replan_cooldown_s, 0.2))

    def _on_target(self, msg):
        new_target = (msg.data or "").strip()
        self.current_target = new_target
        self._bump_sequence_generation(f"target={self.current_target}")
        self._request_stop("target change")
        self._set_phase("folding", reason=f"target update {self.current_target}")
        self.get_logger().info(f"[TARGET] selected={self.current_target} -> restarting from folding")

    def _on_control_state(self, msg: String):
        new_state = (msg.data or "").strip() or "ready"
        if new_state == self.control_state:
            return
        self.control_state = new_state
        if self.control_state in {"paused", "aborted"}:
            self.get_logger().info(f"[GATE] control_state={self.control_state} -> cancel in-flight motion")
            self._bump_sequence_generation(f"control_state={self.control_state}")
            self._request_stop(f"control_state={self.control_state}")
            return
        if self.control_state == "ready":
            self.get_logger().info(f"[GATE] control_state=ready (resume phase={self.phase})")
            return
        self.get_logger().info(f"[GATE] control_state={self.control_state}")

    def _on_arm_armed(self, msg: Bool):
        new_armed = bool(msg.data)
        if new_armed == self.arm_armed:
            return
        self.arm_armed = new_armed
        self.get_logger().info(f"[GATE] arm_armed={self.arm_armed}")
        if not self.arm_armed:
            self._bump_sequence_generation("arm disarmed")
            self._request_stop("arm disarmed")

    # ----------------------
    # Callbacks
    # ----------------------
    def joint_state_cb(self, msg: JointState):
        self.current_joint_state = msg

    def carriage_pos_cb(self, msg: Float64):
        if not self.has_carriage_feedback:
            self.has_carriage_feedback = True
            self.get_logger().info("[INIT] received carriage feedback; using live carriage position for pointing")
        self.current_rail_pos = msg.data

    def _topic_has_publishers(self, subscription) -> bool:
        try:
            return subscription.get_publisher_count() > 0
        except Exception:
            return True

    def _demo_override_active(self, *, topic: str, enabled: bool, subscription, now: float) -> bool:
        if not enabled:
            return False
        if self._topic_has_publishers(subscription):
            return False
        if now - self._started_at_s < self.demo_override_grace_s:
            return False
        key = f"{topic}"
        if key not in self._demo_override_logged:
            self._demo_override_logged.add(key)
            self.get_logger().warn(
                f"[GATE] demo override enabled for {topic}: no publishers after "
                f"{self.demo_override_grace_s:.1f}s grace, treating gate as satisfied"
            )
        return True

    def _effective_ready(self, now: float):
        if self.control_state == "ready":
            return True, "control_state=ready"
        if self._demo_override_active(
            topic="/robot_control_state",
            enabled=self.demo_auto_ready,
            subscription=self._control_state_sub,
            now=now,
        ):
            return True, "demo_auto_ready"
        return False, f"control_state={self.control_state}"

    def _effective_armed(self, now: float):
        if self.arm_armed:
            return True, "arm_armed=true"
        if self._demo_override_active(
            topic="/arm_armed",
            enabled=self.demo_auto_armed,
            subscription=self._arm_armed_sub,
            now=now,
        ):
            return True, "demo_auto_armed"
        return False, "arm_armed=false"

    def _log_blocked(self, now: float, reason: str) -> None:
        if reason == self._last_blocked_reason and (
            now - self._last_blocked_log_s
        ) < self._blocked_log_repeat_s:
            return
        self._last_blocked_reason = reason
        self._last_blocked_log_s = now
        self.get_logger().info(f"[GATE] blocked: {reason}")

    def _consume_backend_outcome(self, now: float) -> None:
        outcome = self.arm_backend.consume_outcome()
        if outcome is None:
            return

        if outcome.context_id != self._sequence_generation:
            self.get_logger().info(
                f"[MOVEIT] ignoring stale outcome request_id={outcome.request_id} "
                f"context={outcome.context_id} current_context={self._sequence_generation} "
                f"phase={outcome.phase} status={outcome.status}"
            )
            self._cooldown_until = max(self._cooldown_until, now + self.replan_cooldown_s)
            return

        if outcome.status == "succeeded":
            if outcome.phase == "folding":
                self._set_phase("turning", reason="fold move complete")
            elif outcome.phase == "turning":
                self._set_phase("stretching", reason="turn move complete")
            elif outcome.phase == "stretching":
                self._set_phase("idle", reason="stretch move complete")
                self._trigger_post_stretch_gesture("stretch move complete")
                self.get_logger().info("[RESULT] pointing sequence complete")
            self._cooldown_until = max(self._cooldown_until, now + 0.05)
            return

        if outcome.status == "canceled":
            self.get_logger().info(
                f"[RESULT] canceled phase={outcome.phase} request_id={outcome.request_id}"
            )
        else:
            self.get_logger().warn(
                f"[RESULT] failed phase={outcome.phase} request_id={outcome.request_id}; "
                f"will retry after cooldown (error={outcome.error_name}, code={outcome.error_code})"
            )

        self._cooldown_until = max(self._cooldown_until, now + self.replan_cooldown_s)

    def _phase_goal_desired_positions(self, phase: str, current_positions, yaw_cmd: float):
        desired = list(current_positions)
        if phase == "folding":
            desired[0] = current_positions[0]
            desired[1:] = [float(v) for v in self.folded_pose[1:]]
            return desired
        if phase == "turning":
            desired[0] = float(yaw_cmd)
            desired[1:] = [float(v) for v in self.folded_pose[1:]]
            return desired
        if phase == "stretching":
            desired[0] = float(yaw_cmd)
            desired[1:] = [float(v) for v in self.stretched_pose[1:]]
            return desired
        return None

    def _turning_yaw_tolerance_rad(self) -> float:
        if self.joint1_deadband_rad > 0.0:
            return max(1e-4, min(self.yaw_deadband_rad, self.joint1_deadband_rad))
        return max(1e-4, self.yaw_deadband_rad)

    def _phase_is_already_satisfied(self, phase: str, current_positions, yaw_cmd: float) -> bool:
        if phase == "folding":
            fold_err = max(
                abs(float(self.folded_pose[i]) - float(current_positions[i]))
                for i in range(1, len(ARM_JOINT_NAMES))
            )
            return fold_err < self.fold_tolerance_rad

        if phase == "turning":
            fold_ok = self._phase_is_already_satisfied("folding", current_positions, yaw_cmd)
            yaw_ok = (
                abs(shortest_angle_diff(yaw_cmd, current_positions[0]))
                < self._turning_yaw_tolerance_rad()
            )
            return fold_ok and yaw_ok

        if phase == "stretching":
            stretch_err = max(
                abs(float(self.stretched_pose[i]) - float(current_positions[i]))
                for i in range(1, len(ARM_JOINT_NAMES))
            )
            yaw_ok = abs(shortest_angle_diff(yaw_cmd, current_positions[0])) < self.yaw_deadband_rad
            return stretch_err < self.fold_tolerance_rad and yaw_ok

        return False

    def _advance_phase_if_already_satisfied(self, current_positions, yaw_cmd: float) -> None:
        progressed = True
        while progressed and self.phase in {"folding", "turning", "stretching"}:
            progressed = False
            if not self._phase_is_already_satisfied(self.phase, current_positions, yaw_cmd):
                return
            if self.phase == "folding":
                self._set_phase("turning", reason="already folded")
                progressed = True
            elif self.phase == "turning":
                yaw_err = abs(shortest_angle_diff(yaw_cmd, current_positions[0]))
                self.get_logger().info(
                    f"[TURN] already at yaw: err={yaw_err:.4f} rad "
                    f"(tol={self._turning_yaw_tolerance_rad():.4f}) "
                    f"current_joint1={float(current_positions[0]):.4f} target_joint1={float(yaw_cmd):.4f}"
                )
                self._set_phase("stretching", reason="already at yaw with folded arm")
                progressed = True
            elif self.phase == "stretching":
                self._set_phase("idle", reason="already stretched and aligned")
                self._trigger_post_stretch_gesture("already stretched and aligned")
                self.get_logger().info("[SEQUENCE] pointing sequence complete (no-op)")
                progressed = False

    # ----------------------
    # Core logic
    # ----------------------
    def update_joint1(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._consume_backend_outcome(now)

        ready_ok, ready_reason = self._effective_ready(now)
        if not ready_ok:
            if not self._topic_has_publishers(self._control_state_sub):
                self._log_blocked(now, f"{ready_reason}; waiting for /robot_control_state publisher")
            else:
                self._log_blocked(now, ready_reason)
            return

        armed_ok, armed_reason = self._effective_armed(now)
        if not armed_ok:
            if not self._topic_has_publishers(self._arm_armed_sub):
                self._log_blocked(now, f"{armed_reason}; waiting for /arm_armed publisher")
            else:
                self._log_blocked(now, armed_reason)
            return

        if self.arm_backend.is_busy():
            return

        if self.current_joint_state is None:
            self._log_blocked(now, "waiting for /joint_states")
            return

        if self.current_target is None:
            if not self._topic_has_publishers(self._target_sub):
                self._log_blocked(now, "waiting for /selected_target publisher")
            else:
                self._log_blocked(now, "waiting for /selected_target command")
            return  # wait until we have a target

        if self.current_target not in TARGET_POSITIONS:
            self.get_logger().warn(f"Unknown target: {self.current_target}")
            self._cooldown_until = max(self._cooldown_until, now + self.replan_cooldown_s)
            return

        if self.phase not in {"folding", "turning", "stretching"}:
            return

        target_pos = TARGET_POSITIONS[self.current_target]
        yaw_angle = self.rail.calculate_yaw_to_target(
            current_pos_raw=self.current_rail_pos,
            target_pos_raw=target_pos,
        )

        name_to_position = dict(zip(self.current_joint_state.name, self.current_joint_state.position))
        missing_joints = [j for j in ARM_JOINT_NAMES if j not in name_to_position]
        if missing_joints:
            if not self._missing_joints_warned:
                self.get_logger().warn(f"Missing joints in /joint_states: {missing_joints}")
                self._missing_joints_warned = True
            return
        self._missing_joints_warned = False

        current_positions = [float(name_to_position[j]) for j in ARM_JOINT_NAMES]
        current_joint1 = current_positions[0]
        ref = self._last_sent_yaw if self._last_sent_yaw is not None else current_joint1
        yaw_cmd = ref + shortest_angle_diff(yaw_angle, ref)

        self._advance_phase_if_already_satisfied(current_positions, yaw_cmd)
        if self.phase not in {"folding", "turning", "stretching"}:
            return

        if now < self._cooldown_until:
            return

        desired = self._phase_goal_desired_positions(self.phase, current_positions, yaw_cmd)
        if desired is None:
            return

        if self.phase == "turning":
            yaw_delta = shortest_angle_diff(yaw_cmd, current_joint1)
            target_str = ", ".join(f"{float(v):.4f}" for v in desired)
            self.get_logger().info(
                f"[TURN] dispatch prep: current_joint1={current_joint1:.4f} target_joint1={yaw_cmd:.4f} "
                f"delta={yaw_delta:.4f} rad (tol={self._turning_yaw_tolerance_rad():.4f}, "
                f"yaw_deadband={self.yaw_deadband_rad:.4f}) targets=[{target_str}]"
            )

        stretch_constraint = None
        if self.phase == "stretching" and self.constrain_joint1_during_stretching:
            stretch_constraint = (
                "joint_1",
                yaw_cmd,
                max(self.stretch_joint1_path_tolerance_rad, self.yaw_deadband_rad),
            )

        sent = self.arm_backend.send_joint_goal(
            phase=self.phase,
            context_id=self._sequence_generation,
            desired_positions=desired,
            start_joint_state=self.current_joint_state,
            stretch_joint1_path_constraint=stretch_constraint,
        )
        if sent:
            self._last_sent_yaw = yaw_cmd
            self._cooldown_until = now + self.replan_cooldown_s
            self._last_blocked_reason = None
        else:
            self._cooldown_until = now + self.replan_cooldown_s


def main(args=None):
    rclpy.init(args=args)
    node = PointJoint1Node()
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
