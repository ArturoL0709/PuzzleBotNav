import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class SignalProcessor(Node):
    def __init__(self):
        super().__init__('signal_processor')
        self.subscription = self.create_subscription(
            Float32, 'signal/data', self.process_signal, 10)
        self.publisher_ = self.create_publisher(Float32, 'proc_signal/data', 10)

    def process_signal(self, msg):
        processed_msg = Float32()
        processed_msg.data = msg.data * 2  # Doblar la amplitud
        self.publisher_.publish(processed_msg)
        self.get_logger().info(f'Processed signal: {processed_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SignalProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
