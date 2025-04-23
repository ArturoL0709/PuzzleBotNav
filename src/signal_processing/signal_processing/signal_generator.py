import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class SignalGenerator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.publisher_ = self.create_publisher(Float32, 'signal/data', 10)
        self.timer = self.create_timer(0.1, self.publish_signal)
        self.t = 0.0

    def publish_signal(self):
        msg = Float32()
        msg.data = np.sin(self.t)  # Señal senoidal
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published signal: {msg.data}')
        self.t += 0.1  # Incremento del tiempo

def main(args=None):
    rclpy.init(args=args)
    node = SignalGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
