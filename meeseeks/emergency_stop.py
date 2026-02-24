#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
import sys, termios, tty

class HardFreezeNode(Node):
    def __init__(self):
        super().__init__('hard_freeze_node')
        self.traj_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.voice_pub = self.create_publisher(String, '/voice_commands', 10)
        self.js_sub = self.create_subscription(JointState, '/joint_states', self._js_cb, 10)
        
        self.last_js = None
        self.get_logger().warn('HARD FREEZE NODE: Drücke [LEERTASTE] für Not-Halt!')

    def _js_cb(self, msg):
        self.last_js = msg

    def freeze_now(self):
        # 1. Den Logik-Status auf Abort setzen
        msg = String()
        msg.data = 'abort'
        self.voice_pub.publish(msg)

        # 2. Hardware-Freeze: Aktuelle Position als Ziel schicken
        if self.last_js:
            traj = JointTrajectory()
            traj.joint_names = self.last_js.name
            p = JointTrajectoryPoint()
            p.positions = self.last_js.position
            p.time_from_start.nanosec = 10000000 # Sofort (0.01s)
            traj.points.append(p)
            self.traj_pub.publish(traj)
            self.get_logger().error('!!! HARD FREEZE: Aktuelle Position fixiert !!!')

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    rclpy.init()
    node = HardFreezeNode()
    try:
        while rclpy.ok():
            if getch() == ' ':
                node.freeze_now()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()