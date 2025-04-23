import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

def normalize_angle(angle):
    """Normaliza un ángulo a estar en el rango [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi

class SquarePathDynamic(Node):
    def __init__(self):
        super().__init__('square_path_dynamic')
        # Definir vértices de un cuadrado de 2 m de lado
        self.vertices = [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        self.current_target_index = 0  # Índice del vértice actual
        self.state = 'move_to_vertex'  # Estados: move_to_vertex, pause y stop
        self.pause_duration = 2.0      # Segundos de pausa en cada vértice
        self.pause_start_time = None

        # Parámetros del controlador
        self.linear_speed = 0.2        # Velocidad lineal constante (m/s)
        self.k_p = 1.5                 # Ganancia para control angular
        self.max_angular_speed = 0.5   # Velocidad angular máxima (rad/s)
        self.distance_threshold = 0.05 # Umbral para considerar que se alcanzó el vértice (m)

        # Odometría teórica (se asume sin error inicial)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_update_time = self.get_clock().now()

        # Publicador de comandos de movimiento
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Timer: cada 20 ms se ejecuta el callback
        timer_period = 0.02  # 20 ms
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Nodo SquarePathDynamic iniciado.")

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time.nanoseconds - self.last_update_time.nanoseconds) / 1e9
        self.last_update_time = current_time

        twist = Twist()
        
        if self.state == 'move_to_vertex':
            # Obtener vértice actual objetivo
            target = self.vertices[self.current_target_index]
            target_x, target_y = target
            # Calcular la distancia y el ángulo deseado
            distance = math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
            desired_angle = math.atan2(target_y - self.y, target_x - self.x)
            angle_error = normalize_angle(desired_angle - self.theta)
            # Control proporcional para la velocidad angular
            angular_speed = self.k_p * angle_error
            # Saturar la velocidad angular
            angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))
            # Publicar comando: avanzar con corrección angular
            twist.linear.x = self.linear_speed
            twist.angular.z = angular_speed
            self.cmd_vel_pub.publish(twist)
            # Actualizar la odometría teórica (integración simple)
            self.x += self.linear_speed * math.cos(self.theta) * dt
            self.y += self.linear_speed * math.sin(self.theta) * dt
            self.theta += angular_speed * dt
            self.theta = normalize_angle(self.theta)
            # Verificar si se alcanzó el vértice
            if distance < self.distance_threshold:
                self.get_logger().info(
                    f"Vértice {self.current_target_index+1} alcanzado: ({target_x:.2f}, {target_y:.2f}). " +
                    f"Ángulo deseado: {desired_angle:.2f}, Ángulo actual: {self.theta:.2f}, Error: {angle_error:.2f}"
                )
                # Entrar en estado de pausa
                self.state = 'pause'
                self.pause_start_time = current_time
                # Publicar comando de detención
                stop_twist = Twist()
                self.cmd_vel_pub.publish(stop_twist)

        elif self.state == 'pause':
            # Durante la pausa, se detiene el robot
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            elapsed_pause = (current_time.nanoseconds - self.pause_start_time.nanoseconds) / 1e9
            if elapsed_pause >= self.pause_duration:
                self.current_target_index += 1
                if self.current_target_index >= len(self.vertices):
                    self.get_logger().info("Trayectoria cuadrada completada.")
                    self.state = 'stop'
                else:
                    self.get_logger().info(
                        f"Moviéndose hacia el vértice {self.current_target_index+1}: {self.vertices[self.current_target_index]}"
                    )
                    self.state = 'move_to_vertex'

        elif self.state == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SquarePathDynamic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
