import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import py_pubsub.voice as transcriber
import threading


class MinimalPublisher(Node):

   def __init__(self):
        super().__init__('publisher')
        self.publisher_ = self.create_publisher(String, '/voice_commands', 10)
        
        # Start transcriber threads in background
        t = threading.Thread(target=transcriber.main, daemon=True)
        t.start()
        
        # Timer just checks the queue
        self.timer_ = self.create_timer(0.1, self.timer_callback)

   def timer_callback(self):
        try:
            text = transcriber.transcription_queue.get_nowait()
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{text}"')
        except transcriber.transcription_queue.Empty:
            pass 


def main(args=None):
   rclpy.init(args=args)

   minimal_publisher = MinimalPublisher()

   rclpy.spin(minimal_publisher)

   minimal_publisher.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()