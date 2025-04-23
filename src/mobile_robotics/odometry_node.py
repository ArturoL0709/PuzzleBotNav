#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from example_interfaces.srv import Trigger

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')
        # Estado interno
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Publishers
        self.pose_pub = self.create_publisher(Pose2D, 'pose', 10)

        # Subscribers
        self.create_subscription(Float32, 'VelocityEncR', self.r_callback, 10)
        self.create_subscription(Float32, 'VelocityEncL', self.l_callback, 10)

        # Servicio de reset
        self.create_service(Trigger, 'reset_odometry', self.reset_callback)

        # Timer para publicar pose periódicamente
        self.create_timer(0.1, self.publish_pose)

    def r_callback(self, msg: Float32):
        # TODO: almacenar velocidad de rueda derecha
        pass

    def l_callback(self, msg: Float32):
        # TODO: almacenar velocidad de rueda izquierda
        pass

    def reset_callback(self, request, response):
        # Reinicia la pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        response.success = True
        response.message = 'Odometry reset'
        return response

    def publish_pose(self):
        pose = Pose2D()
        pose.x = self.x
        pose.y = self.y
        pose.theta = self.theta
        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
