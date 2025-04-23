import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import math
from tf_transformations import quaternion_from_euler

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        self.get_logger().info("✅ OdometryNode iniciado y timer a 50 Hz OK")

        
        # Parámetros
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_radius', 0.05),
                ('wheel_base', 0.18),
                ('odom_frame', 'odom'),
                ('base_frame', 'base_link'),
            ]
        )
        
        # Servicio de reset
        self.reset_srv = self.create_service(Empty, 'reset_odometry', self.reset_callback)
        
        # Suscriptores
        self.create_subscription(Float32, 'VelocityEncR', self.encR_callback, 10)
        self.create_subscription(Float32, 'VelocityEncL', self.encL_callback, 10)
        
        # Publicadores
        self.pose2d_pub = self.create_publisher(Pose2D, 'pose', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)
        
        # Variables
        self.encR = 0.0
        self.encL = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        # justo después de inicializar variables…
        self.last_time = self.get_clock().now()
        #crea un timer que llame a update_pose() a 50 Hz
        self.create_timer(0.02, self.update_pose)


    def reset_callback(self, request, response):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.get_logger().info("Odometría reiniciada.")
        return response  # <-- ¡Corregido!

    def encR_callback(self, msg):
        self.encR = msg.data

    def encL_callback(self, msg):
        self.encL = msg.data

    def update_pose(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        self.last_time = now

        # Cálculos
        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_base').value
        v = r * (self.encR + self.encL) / 2.0
        w = r * (self.encR - self.encL) / L

        # Integración
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = self.normalize_angle(self.theta + w * dt)

        # Publicar Pose2D
        pose2d = Pose2D()
        pose2d.x = self.x
        pose2d.y = self.y
        pose2d.theta = self.theta
        self.pose2d_pub.publish(pose2d)

        # Publicar Odometry (opcional)
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

    @staticmethod
    def normalize_angle(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()