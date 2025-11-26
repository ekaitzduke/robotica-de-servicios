import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import Int8 # Usaremos Int8 para mandar números desde el teclado
from geometry_msgs.msg import Twist

# Definimos los estados posibles
class RobotState(Enum):
    WANDER = 1
    APPROACH = 2
    RECOGNIZE = 3
    EXECUTE = 4

# Gestos posibles (mapeo a números de teclado)
class Gesture(Enum):
    NONE = 0
    CALL_ROBOT = 1           # Mano levantada (1)
    ORDER_COMPLETE = 2       # Pulgar arriba (2)
    SELECT_ORDER = 3         # N dedos levantados (3)
    ASK_FOR_BILL = 4         # Apuntar con dedos (4)
    COMPLAINT_FORM = 5       # Pulgar abajo (5)
    ASK_WIFI = 6             # Gesto teléfono (6)

class WaiterRobot(Node):
    def __init__(self):
        super().__init__('waiter_robot')

        self.state = RobotState.WANDER
        self.current_gesture = Gesture.NONE
        self.steps_counter = 0
        
        # 1. CREAR SUSCRIPTOR: Escucha en el topic /comando_gesto
        self.subscription = self.create_subscription(Int8, '/comando_gesto', self.gesture_callback, 10)                             
        self.subscription # Previene warning de variable no usada

        # 2. PUBLICADOR DE VELOCIDAD (Motores)
        # Publicamos en /cmd_vel_unstamped. El nodo de TB4 lo convertirá a /cmd_vel con timestamp.
        # Esta es la forma correcta de interactuar con la arquitectura del TB4.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        # 2. TEMPORIZADOR para la máquina de estados
        self.create_timer(0.1, self.state_machine_step)
        self.get_logger().info("Robot camarero iniciado. Estado inicial: WANDER")
        self.get_logger().info("Esperando comandos en el topic /comando_gesto (1-6)")


    def gesture_callback(self, msg):
        """Función que se activa al recibir un comando por teclado."""
        try:
            # Convierte el número recibido (msg.data) al Enum Gesture correspondiente
            self.current_gesture = Gesture(msg.data)
            self.get_logger().info(f"⌨️ Comando recibido: {self.current_gesture.name} ({msg.data})")
        except ValueError:
            self.get_logger().warn(f"⚠️ Comando inválido: {msg.data}. Debe ser un número del 1 al 6.")
            self.current_gesture = Gesture.NONE

    # --- Lógica de la Máquina de Estados (Mismo código que tenías) ---
    def state_machine_step(self):
        """Ejecuta un ciclo de la máquina de estados."""
        # Solo procesamos la máquina de estados si estamos en WANDER o RECOGNIZE
        if self.state == RobotState.WANDER:
            self.wander()
        elif self.state == RobotState.APPROACH:
            self.approach()
        elif self.state == RobotState.RECOGNIZE:
            self.recognize()
        elif self.state == RobotState.EXECUTE:
            self.execute()

    def wander(self):
        # LÓGICA: Dar vueltas en círculo (Avanzar + Girar)
        msg = Twist()
        msg.linear.x = 0.3   # Velocidad hacia adelante
        msg.angular.z = 0.5  # Velocidad de giro
        self.cmd_vel_pub.publish(msg)

        # Transición si recibimos un 1
        if self.current_gesture == Gesture.CALL_ROBOT:
            self.get_logger().info("✋ 'CALL_ROBOT' recibido -> Acercándose...")
            self.state = RobotState.APPROACH
            self.steps_counter = 0 # Reseteamos el contador de pasos
            self.current_gesture = Gesture.NONE

    def approach(self):
        # LÓGICA: Avanzar recto una distancia corta (durante 40 ciclos = 4 segundos)
        limit_steps = 40 
        
        if self.steps_counter < limit_steps:
            msg = Twist()
            msg.linear.x = 0.5  # Avanzar más rápido
            msg.angular.z = 0.0 # Sin girar
            self.cmd_vel_pub.publish(msg)
            self.steps_counter += 1
        else:
            # Cuando termina la distancia, frenamos y cambiamos de estado
            self.stop_robot()
            self.get_logger().info("🛑 Llegada al cliente. Esperando orden (RECOGNIZE).")
            self.state = RobotState.RECOGNIZE
    
    def recognize(self):
        # LÓGICA: Estar quieto hasta recibir orden
        self.stop_robot() # Aseguramos que esté quieto

        # Esperamos cualquier gesto válido (del 2 al 6)
        if self.current_gesture != Gesture.NONE and self.current_gesture != Gesture.CALL_ROBOT:
            self.get_logger().info(f"✅ Gesto {self.current_gesture.name} entendido.")
            self.state = RobotState.EXECUTE

    def execute(self):
        """Ejecuta acción según el gesto reconocido"""
        gesture = self.current_gesture
        
        if gesture == Gesture.SELECT_ORDER:
            self.get_logger().info("🍽️ Pedido recibido, ejecutando...")
        elif gesture == Gesture.ASK_FOR_BILL:
            self.get_logger().info("💳 Cliente pidió la cuenta.")
        elif gesture == Gesture.COMPLAINT_FORM:
            self.get_logger().info("📄 Entregando hoja de reclamaciones.")
        elif gesture == Gesture.ORDER_COMPLETE:
            self.get_logger().info("✅ Pedido completado, regresando a WANDER.")
        elif gesture == Gesture.ASK_WIFI:
            self.get_logger().info("📶 Entregando clave WiFi.")
        else:
            # Esto maneja el caso de que se haya recibido un 1 (CALL_ROBOT) mientras ya estaba aquí.
            self.get_logger().info("❓ Gesto ambiguo o no ejecutable. Volviendo a WANDER.")

        # Vuelve al estado inicial y limpia el gesto
        self.state = RobotState.WANDER
        self.current_gesture = Gesture.NONE

    # Se elimina la función detect_gesture ya que es reemplazada por el suscriptor

    # CORRECCIÓN: Añadida la función que faltaba
    def stop_robot(self):
        """Envía velocidad cero para detener el robot"""
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WaiterRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()