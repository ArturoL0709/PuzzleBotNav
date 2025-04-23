#!/usr/bin/env python3
import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator')

        # 1) Carga de parámetros desde YAML
        pkg_share = get_package_share_directory('mobile_robotics')
        yaml_file = os.path.join(pkg_share, 'params', 'path_params.yaml')
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        # ojo, aquí tomamos la sección correcta
        params = data['path_generator']['ros__parameters']
        self.waypoints      = params['waypoints']
        self.pause_duration = float(params['pause_duration'])

        # 2) Publicadores y suscriptores
        self.goal_pub    = self.create_publisher(PointStamped, 'goal_pose', 10)
        self.create_subscription(Bool, 'goal_reached', self._reached_cb, 10)
        self.mission_pub = self.create_publisher(Bool, 'mission_complete', 10)

        # 3) Estado de la FSM
        self.current_idx = 0
        self.state       = 'MOVING'    # Arrancamos en MOVING
        self.pause_start = None

        # 4) Publica inmediatamente el primer waypoint
        self._publish_waypoint()

        # 5) Timer para gestionar la FSM (10 Hz)
        self.create_timer(0.1, self._timer_cb)

    def _publish_waypoint(self):
        """Envía el waypoint actual a /goal_pose."""
        x, y = self.waypoints[self.current_idx]
        goal = PointStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.point.x = float(x)
        goal.point.y = float(y)
        goal.point.z = 0.0
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Waypoint {self.current_idx+1} → ({x:.2f}, {y:.2f})')

    def _reached_cb(self, msg: Bool):
        """Cuando ControllerNode publica goal_reached=True, pasamos a PAUSE."""
        if msg.data and self.state == 'MOVING':
            self.state = 'PAUSE'
            self.pause_start = self.get_clock().now()
            self.get_logger().info(
                f'Waypoint {self.current_idx+1} alcanzado, pausando {self.pause_duration}s'
            )

    def _timer_cb(self):
        """FSM: tras PAUSE avanza al siguiente waypoint o termina la misión."""
        if self.state == 'PAUSE':
            elapsed = (self.get_clock().now().nanoseconds 
                       - self.pause_start.nanoseconds) / 1e9
            if elapsed >= self.pause_duration:
                self.current_idx += 1
                if self.current_idx >= len(self.waypoints):
                    # misión completa
                    self.state = 'IDLE'
                    self.get_logger().info('¡Misión completada!')
                    self.mission_pub.publish(Bool(data=True))
                else:
                    # siguiente waypoint
                    self.state = 'MOVING'
                    self._publish_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
