#!/usr/bin/env python3
"""
OdometryNode   ·   PuzzleBot (ROS 2)
------------------------------------
Calcula la pose a partir de /VelocityEncR y /VelocityEncL
y publica /pose  y /odometry  (QoS RELIABLE).
Resetea con /reset_odometry.
"""
import math, rclpy
from rclpy.node import Node
from rclpy.qos  import qos_profile_sensor_data
from std_msgs.msg    import Float32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg      import Odometry
from std_srvs.srv      import Empty
from tf_transformations import quaternion_from_euler


class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # ───────── parámetros sobre-escribibles ─────────
        self.declare_parameters('', [
            ('wheel_radius', 0.05),   # [m]
            ('wheel_base',   0.18),   # [m]
            ('odom_frame',  'odom'),
            ('base_frame',  'base_link')
        ])

        # ───────── suscriptores BEST_EFFORT ─────────
        self.create_subscription(Float32, 'VelocityEncR',
                                 self._enc_r_cb, qos_profile_sensor_data)
        self.create_subscription(Float32, 'VelocityEncL',
                                 self._enc_l_cb, qos_profile_sensor_data)

        self._stamp_r = self._stamp_l = None   # sellos de tiempo

        # ───────── publicadores RELIABLE ─────────
        self.pose_pub = self.create_publisher(Pose2D,   'pose',     10)
        self.odom_pub = self.create_publisher(Odometry, 'odometry', 10)

        # ───────── servicio reset ─────────
        self.create_service(Empty, 'reset_odometry', self._reset_srv)

        # ───────── estado interno ─────────
        self._w_r = self._w_l = 0.0
        self.x = self.y = self.theta = 0.0
        self._last_time = self.get_clock().now()

        # timer 50 Hz
        self.create_timer(0.02, self._update_pose)
        self.get_logger().info('✅ OdometryNode listo (50 Hz)')

    # ---------- callbacks de rueda ----------
    def _enc_r_cb(self, msg: Float32):
        self._w_r = msg.data
        self._stamp_r = self.get_clock().now()

    def _enc_l_cb(self, msg: Float32):
        self._w_l = msg.data
        self._stamp_l = self.get_clock().now()

    def _reset_srv(self, _, res):
        self.x = self.y = self.theta = 0.0
        self._last_time = self.get_clock().now()
        self.get_logger().info('⟳ Odometry reiniciada')
        return res

    # ---------- integración ----------
    def _update_pose(self):
        # Esperamos muestra de ambas ruedas
        if self._stamp_r is None or self._stamp_l is None:
            return

        # mismas “ventana” de 25 ms
        if abs(self._stamp_r.nanoseconds - self._stamp_l.nanoseconds) > 25e6:
            return

        now = self.get_clock().now()
        dt  = (now.nanoseconds - self._last_time.nanoseconds) * 1e-9
        if dt <= 0.0:
            return
        self._last_time = now

        r = self.get_parameter('wheel_radius').value
        L = self.get_parameter('wheel_base').value
        v = r * (self._w_r + self._w_l) / 2.0
        w = r * (self._w_r - self._w_l) / L

        # Euler
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = self._wrap(self.theta + w * dt)

        # --- Pose2D
        pose = Pose2D(x=self.x, y=self.y, theta=self.theta)
        self.pose_pub.publish(pose)

        # --- Odometry
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id  = self.get_parameter('base_frame').value
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, \
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

    # ---------- util ----------
    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2*math.pi) - math.pi


def main():
    rclpy.init()
    rclpy.spin(OdometryNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
