#!/usr/bin/env python3
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from meeseeks.targetSelection import selectNewTarget
import meeseeks.globalVariables as gv

class GestureController:
    def __init__(self, node: Node, cb_group: ReentrantCallbackGroup):
        self._node = node
        self._cli_pause = node.create_client(Trigger, "/gesture/pause", callback_group=cb_group)
        self._cli_resume = node.create_client(Trigger, "/gesture/resume", callback_group=cb_group)
        self._cli_abort = node.create_client(Trigger, "/gesture/abort", callback_group=cb_group)

    def _call(self, client):
        if client.service_is_ready(): client.call_async(Trigger.Request())

    def pause(self): self._call(self._cli_pause)
    def resume(self): self._call(self._cli_resume)
    def abort(self): self._call(self._cli_abort)

class MainLogic(Node):
    def __init__(self):
        super().__init__("main_logic")
        self.cb_group = ReentrantCallbackGroup()
        
        # Publisher
        self.target_pub = self.create_publisher(String, "/selected_target", 10)
        self.control_state_pub = self.create_publisher(String, "/robot_control_state", 10)
        self.traj_pub = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

        # State
        self.state = "initializing"
        self._state_lock = threading.RLock()
        self.current_joint2_pos = 0.0
        
        # Konfiguration für die "virtuelle Fahrt"
        self.arm_move_duration = 8.0  # Wie viele Sekunden der Arm "fährt"
        self.joint2_start = 0.0       # Startwinkel Joint 2
        self.joint2_end = 1.0         # Endwinkel Joint 2 (gestreckt)

        # Subscriptions
        self.create_subscription(String, "/voice_commands", self._on_voice_command, 10, callback_group=self.cb_group)
        
        self.gesture = GestureController(self, self.cb_group)
        self._start_target_cycle(initial=True)
        
        self.control_timer = self.create_timer(0.1, self.main_control_loop, callback_group=self.cb_group)
        self.get_logger().info("CLEAN LOG: MainLogic bereit. Steuerung via Voice/CLI.")

    def _on_voice_command(self, msg: String):
        cmd = msg.data.lower()
        self.get_logger().info(f"\n[VOICE-COMMAND] >>> {cmd.upper()} <<<")
        
        if any(x in cmd for x in ["abort", "stop"]):
            self._handle_abort()
        elif "pause" in cmd:
            self._handle_pause()
        elif "continue" in cmd or "resume" in cmd:
            self._handle_continue()
        elif "where" in cmd:
            self.get_logger().info(f"STATUS: Ziel ist {gv.currentTargetGlobal}. Wir bewegen uns virtuell.")
            if self.state == "waiting_before_move": self._begin_virtual_move()

    def _handle_abort(self):
        with self._state_lock:
            self.state = "aborted"
            self.gesture.abort()
            self.get_logger().error("!!! ABORT !!! - Alle Bewegungen gestoppt.")

    def _handle_pause(self):
        with self._state_lock:
            self.state = "paused"
            self.gesture.pause()
            self.get_logger().warn("|| PAUSE || - Armbewegung angehalten.")

    def _handle_continue(self):
        with self._state_lock:
            if self.state in ["paused", "aborted"]:
                self.state = "ready"
                self.gesture.resume()
                self.get_logger().info(">> CONTINUE << - Bewegung wird fortgesetzt.")

    def main_control_loop(self):
        with self._state_lock:
            if self.state != "ready": return
            
            # Hier simulieren wir die "Fahrt" durch langsames Erhöhen von Joint 2
            step = (self.joint2_end - self.joint2_start) / (self.arm_move_duration * 10) # 10Hz Timer
            
            if self.current_joint2_pos < self.joint2_end:
                self.current_joint2_pos += step
                self._send_arm_pose(self.current_joint2_pos)
            else:
                self._target_reached()

    def _send_arm_pose(self, j2_val):
        traj = JointTrajectory()
        traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        p = JointTrajectoryPoint()
        # Wir lassen Joint 1 aus (das macht der Pointing Node), ändern nur Joint 2
        # Die restlichen Werte auf Standard (1.57 ist oft ein guter Winkel für j3)
        p.positions = [0.0, j2_val, 1.57, 0.0, 0.0, 0.0] 
        p.time_from_start.nanosec = 100000000 # 0.1s
        traj.points.append(p)
        # Hinweis: Hier senden wir eigentlich j1=0.0, was mit dem Pointing Node 
        # kollidieren könnte. Im Lab ist es besser, Joint 1 hier im Trajectory 
        # wegzulassen oder den Pointing Node die ganze Trajektorie bauen zu lassen.
        self.traj_pub.publish(traj)

    def _start_target_cycle(self, initial=bool):
        target = selectNewTarget(None if initial else gv.currentTargetGlobal)
        gv.currentTargetGlobal = target
        
        # Pointing Node informieren
        msg = String()
        msg.data = target
        self.target_pub.publish(msg)
        
        self.state = "waiting_before_move"
        self.current_joint2_pos = self.joint2_start
        
        print("\n" + "-"*40)
        self.get_logger().info(f"NEUER ZYKLUS: Ziel {target}")
        self.get_logger().info("Phase: Warten (Arm zeigt nur)")
        print("-"*40)
        
        # Nach 5s automatisch losfahren
        threading.Timer(5.0, self._begin_virtual_move).start()

    def _begin_virtual_move(self):
        with self._state_lock:
            if self.state == "waiting_before_move":
                self.state = "ready"
                self.get_logger().info("START: Arm bewegt sich jetzt vorwärts zum Ziel...")

    def _target_reached(self):
        self.state = "waiting_after_reach"
        self.get_logger().warn(f"ERREICHT: Ziel {gv.currentTargetGlobal} erreicht.")
        threading.Timer(5.0, lambda: self._start_target_cycle(False)).start()

    def _publish_control_state(self):
        msg = String(); msg.data = self.state
        self.control_state_pub.publish(msg)

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()


