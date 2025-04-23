import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist, PointStamped
from std_msgs.msg import Bool
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Parámetros desde YAML
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Kv', 0.5),
                ('Kw', 1.0),
                ('max_lin_speed', 0.3),
                ('max_ang_speed', 1.0),
                ('distance_tolerance', 0.05),
            ]
        )
        
        # Suscriptores
        self.create_subscription(Pose2D, 'pose', self.pose_callback, 10)
        self.create_subscription(PointStamped, 'goal_pose', self.goal_callback, 10)
        
        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reached_pub = self.create_publisher(Bool, 'goal_reached', 10)
        # tras crear cmd_pub y reached_pub:
        self.create_timer(0.1, self.control_loop)  # corre a 10 Hz

        
        # Estado
        self.current_pose = None
        self.current_goal = None

    def pose_callback(self, msg):
        self.current_pose = msg

    def goal_callback(self, msg):
        self.current_goal = (msg.point.x, msg.point.y)
        self.get_logger().info(f'Nuevo objetivo: {self.current_goal}')

    def control_loop(self):
        if self.current_pose is None or self.current_goal is None:
            return

        xg, yg = self.current_goal
        x = self.current_pose.x
        y = self.current_pose.y
        theta = self.current_pose.theta

        # Errores
        ed = math.hypot(xg - x, yg - y)
        eg = math.atan2(yg - y, xg - x) - theta
        eg = self.normalize_angle(eg)

        # Control
        Kv = self.get_parameter('Kv').value
        Kw = self.get_parameter('Kw').value
        v = Kv * ed
        w = Kw * eg

        # Saturación
        v = min(v, self.get_parameter('max_lin_speed').value)
        w = max(-self.get_parameter('max_ang_speed').value, 
                min(w, self.get_parameter('max_ang_speed').value))

        # Publicar
        twist = Twist()
        if ed > self.get_parameter('distance_tolerance').value:
            twist.linear.x = v
            twist.angular.z = w
        else:
            twist = Twist()  # Asegurar parada total
            self.reached_pub.publish(Bool(data=True))
            self.current_goal = None
        self.cmd_pub.publish(twist)

    @staticmethod
    def normalize_angle(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    controller_node = ControllerNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller_node)
    executor.spin()
    controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()