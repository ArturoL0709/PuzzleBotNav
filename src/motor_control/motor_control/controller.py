# Imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult

# Clase Controlador PID
class Controller(Node):
    def __init__(self):
        super().__init__('ctrl')

        # Declarar parámetros
        self.declare_parameter('Kp', 1.0)  # Ganancia proporcional
        self.declare_parameter('Ki', 0.0)  # Ganancia integral
        self.declare_parameter('Kd', 0.0)  # Ganancia derivativa

        # Leer parámetros
        self.Kp = self.get_parameter('Kp').value
        self.Ki = self.get_parameter('Ki').value
        self.Kd = self.get_parameter('Kd').value

        # Variables del controlador
        self.prev_error = 0.0
        self.integral = 0.0
        self.dt = 0.01  # Tiempo de muestreo (igual al del motor)

        # Suscribirse a los temas de set point y velocidad del motor
        self.sub_setpoint = self.create_subscription(Float32, 'set_point', self.setpoint_callback, 10)
        self.sub_motor_speed = self.create_subscription(Float32, 'motor_speed_y', self.speed_callback, 10)

        # Publicador al motor
        self.pub_motor = self.create_publisher(Float32, 'motor_input_u', 10)

        # Timer para calcular el control cada dt segundos
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Callback para cambiar parámetros en tiempo real
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Variables de entrada
        self.setpoint = 0.0
        self.motor_speed = 0.0

        self.get_logger().info("Controlador PID iniciado 🚀")

    def setpoint_callback(self, msg):
        """ Recibe el set point deseado """
        self.setpoint = msg.data

    def speed_callback(self, msg):
        """ Recibe la velocidad actual del motor """
        self.motor_speed = msg.data

    def control_loop(self):
        """ Calcula la señal de control y la envía al motor """
        error = self.setpoint - self.motor_speed  # Calcular error

        # Control proporcional
        P = self.Kp * error

        # Control integral
        self.integral += error * self.dt
        I = self.Ki * self.integral

        # Control derivativo
        derivative = (error - self.prev_error) / self.dt
        D = self.Kd * derivative

        # Señal de control
        control_signal = P + I + D

        # Limitar la señal de control (evitar valores extremos)
        control_signal = max(min(control_signal, 12.0), -12.0)  # Suponiendo un rango de -12V a 12V

        # Publicar la señal de control
        msg = Float32()
        msg.data = control_signal
        self.pub_motor.publish(msg)

        # Guardar error anterior
        self.prev_error = error

    def parameters_callback(self, params):
        """ Callback para actualizar parámetros en tiempo real """
        for param in params:
            if param.name == "Kp":
                self.Kp = param.value
                self.get_logger().info(f"Kp actualizado a {self.Kp}")
            elif param.name == "Ki":
                self.Ki = param.value
                self.get_logger().info(f"Ki actualizado a {self.Ki}")
            elif param.name == "Kd":
                self.Kd = param.value
                self.get_logger().info(f"Kd actualizado a {self.Kd}")

        return SetParametersResult(successful=True)


# Main
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
