#!/usr/bin/env python3
import sys
import time
import threading
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Empty


ARM_JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


class EmergencyStop(Node):
    def __init__(self):
        super().__init__("emergency_stop")

        # Publishers (match your CLI commands)
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        self.carriage_stop_pub = self.create_publisher(Empty, "/elmo/id2/carriage/stop", 10)
        self.lift_stop_pub = self.create_publisher(Empty, "/elmo/id2/lift/stop", 10)

        # Joint state subscriber (to hold current position)
        self._last_joint_state = None
        self._js_lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)

        # Keyboard thread
        self._running = True
        self._kb_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._kb_thread.start()

        self.get_logger().info(
            "EmergencyStop ready. Press SPACE or 'e' to STOP. Press 'q' to quit."
        )

    def _on_joint_state(self, msg: JointState):
        with self._js_lock:
            self._last_joint_state = msg

    def _get_current_arm_positions(self):
        with self._js_lock:
            msg = self._last_joint_state
        if msg is None:
            return None

        name_to_pos = {n: p for n, p in zip(msg.name, msg.position)}
        positions = []
        for j in ARM_JOINT_NAMES:
            if j not in name_to_pos:
                return None
            positions.append(float(name_to_pos[j]))
        return positions

    def trigger_estop(self):
        # 1) Stop carriage + lift
        empty = Empty()
        self.carriage_stop_pub.publish(empty)
        self.lift_stop_pub.publish(empty)

        # 2) Stop/hold arm by commanding current positions
        positions = self._get_current_arm_positions()
        if positions is None:
            self.get_logger().warn(
                "E-STOP pressed, but cannot read all arm joints from /joint_states yet. "
                "Carriage/lift stop published; arm hold skipped."
            )
            return

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = positions
        # Very short horizon to “grab” the controller and hold immediately
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = 200_000_000  # 0.2s

        traj.points = [pt]
        self.arm_pub.publish(traj)

        self.get_logger().error("🚨 E-STOP SENT: arm hold + carriage stop + lift stop")

    def _keyboard_loop(self):
        # Non-blocking single-key input (no extra deps)
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)  # immediate keypresses
            while self._running and rclpy.ok():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not r:
                    continue
                ch = sys.stdin.read(1)
                if not ch:
                    continue

                ch_lower = ch.lower()
                if ch == " " or ch_lower == "e":
                    self.trigger_estop()
                elif ch_lower == "q":
                    self.get_logger().info("Quitting EmergencyStop.")
                    self._running = False
                    # Shutdown ROS from the node thread
                    rclpy.shutdown()
                    return
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    node = EmergencyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()