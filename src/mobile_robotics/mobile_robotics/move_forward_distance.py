import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Twist  
from std_msgs.msg import Float32

class MoveForwardClass(Node): 

    def __init__(self): 
        super().__init__('move_forward')  # Inicia el nodo con el nombre "move_forward"
        # Declaración de variables necesarias
        self.start_time = self.get_clock().now()  # Momento en que el robot empieza a moverse  
        self.stop_time = 10.0  # Tiempo en segundos para detener el robot
        self.state = "stop"  # Estado inicial: detenido
        self.first_time = True 
        self.d = 0.0  # Distancia a mover (m)
        self.v = 0.2  # Velocidad (m/s)
        self.t = 0.0  # Tiempo de movimiento (s)
        self.distance_received = False
        # Inicializa el suscriptor de ROS
        self.distance_sub = self.create_subscription(
            Float32,
            "distance",
            self.distance_cb,  # Se usa el nombre correcto de la función callback
            10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10) 
        # Declara el temporizador de ROS
        timer_period = 0.05  # Periodo en segundos para llamar a la función timer_callback
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        self.get_logger().info("Node initialized!!!") 
        self.vel = Twist() 

    def distance_cb(self, distance_smg):
        # Extrae la componente data del mensaje Float32 que indica la distancia a mover (m)
        self.d = distance_smg.data
        self.t = self.d / self.v  # Calcula el tiempo necesario para mover la distancia deseada
        self.distance_received = True

    def timer_callback(self): 
        if self.state == "stop": 
            self.vel.linear.x = 0.0  # m/s 
            self.vel.angular.z = 0.0  # rad/s 
            self.cmd_vel_pub.publish(self.vel)  # Publica el mensaje de detenerse 
            if self.distance_received: 
                self.distance_received = False 
                self.state = "move_linear"  # Cambia el estado para mover el robot hacia adelante 
                self.start_time = self.get_clock().now()  # Actualiza el tiempo de inicio de movimiento 
                self.get_logger().info("Change to move linear") 

        elif self.state == "move_linear": 
            self.vel.linear.x = self.v  # m/s 
            self.vel.angular.z = 0.0  # rad/s 
            self.cmd_vel_pub.publish(self.vel)  # Publica el mensaje de avance 
            # Si ya se ha cumplido el tiempo necesario para mover la distancia deseada, cambia el estado a detenerse
            if self.get_clock().now().nanoseconds - self.start_time.nanoseconds >= int(self.t * 10**9):
                self.state = "stop"
                self.get_logger().info("Stopping")

def main(args=None): 
    rclpy.init(args=args) 
    move_publisher = MoveForwardClass() 
    rclpy.spin(move_publisher) 
    move_publisher.destroy_node() 
    rclpy.shutdown() 

if __name__ == '__main__': 
    main() 
