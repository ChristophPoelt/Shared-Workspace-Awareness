#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        
        # 1. Publisher für die Logik (damit MainLogic auf 'aborted' springt)
        self.logic_pub = self.create_publisher(String, '/voice_commands', 10)
        
        # 2. Publisher für die Hardware (überschreibt laufende Bewegungen)
        self.traj_pub = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10)
        
        # 3. Subscriber für die aktuelle Position
        self.js_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self._joint_state_callback, 
            10)
        
        self.current_js = None
        self.get_logger().warn('--- EMERGENCY STOP NODE READY ---')
        self.get_logger().info('DRÜCKE [LEERTASTE] FÜR SOFORTIGEN HARD-STOP!')

    def _joint_state_callback(self, msg):
        """Speichert die absolut letzte bekannte Position des Roboters."""
        self.current_js = msg

    def trigger_hard_stop(self):
        # Schritt A: Logik stoppen
        logic_msg = String()
        logic_msg.data = 'abort'
        self.logic_pub.publish(logic_msg)
        
        # Schritt B: Hardware einfrieren
        if self.current_js:
            stop_traj = JointTrajectory()
            stop_traj.joint_names = self.current_js.name
            
            point = JointTrajectoryPoint()
            # Wir befehlen dem Roboter die IST-Position als SOLL-Position
            point.positions = self.current_js.position
            # Wir geben 0 Sekunden Zeit -> Sofortiger Stopp
            point.time_from_start.nanosec = 1 
            
            stop_traj.points.append(point)
            self.traj_pub.publish(stop_traj)
            
            self.get_logger().error('!!! EMERGENCY STOP: Hardware eingefroren !!!')
        else:
            self.get_logger().error('STOP FEHLGESCHLAGEN: Keine JointStates empfangen!')

def getch():
    """Liest ein einzelnes Zeichen direkt vom Terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()

    try:
        while rclpy.ok():
            char = getch()
            # Leertaste (' ')
            if char == ' ':
                node.trigger_hard_stop()
            # 'q' zum Beenden des Skripts
            elif char.lower() == 'q':
                break
            
            # Kurz ROS-Callbacks verarbeiten (wichtig für den JointState-Subscriber)
            rclpy.spin_once(node, timeout_sec=0.01)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()