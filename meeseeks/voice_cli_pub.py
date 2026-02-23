#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


HELP = """\
Type a command and press Enter, e.g.:
  target
  pause
  continue
  resume
  abort
  where are you going
Other:
  help
  exit / quit
"""


class VoiceCliPublisher(Node):
    def __init__(self):
        super().__init__("voice_cli_publisher")
        self.pub = self.create_publisher(String, "/voice_commands", 10)
        self.get_logger().info("Voice CLI Publisher started. Publishing to /voice_commands")
        self.get_logger().info(HELP)

    def publish_text(self, text: str):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"[CLI->VOICE] published: '{text}'")


def main():
    rclpy.init()
    node = VoiceCliPublisher()

    try:
        while rclpy.ok():

            try:
                line = input("> ").strip()
            except EOFError:
                break

            if not line:
                continue

            low = line.lower().strip()
            if low in ("exit", "quit", "q"):
                break
            if low in ("help", "h", "?"):
                print(HELP)
                continue

            node.publish_text(line)

            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()