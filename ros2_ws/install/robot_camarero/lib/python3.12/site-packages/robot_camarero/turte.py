import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import Int8 # Usaremos Int8 para mandar números desde el teclado

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
        # CORRECCIÓN: Doble guion bajo
        super().__init__('waiter_robot')
        self.state = RobotState.WANDER
        self.current_gesture = Gesture.NONE
        
        # 1. CREAR SUSCRIPTOR: Escucha en el topic /comando_gesto
        self.subscription = self.create_subscription(
            Int8,                           # Tipo de mensaje: Entero de 8 bits
            '/comando_gesto',               # Nombre del topic
            self.gesture_callback,          # Función que se llama al recibir un mensaje
            10)                             # Calidad de Servicio (QoS)
        self.subscription # Previene warning de variable no usada

        # 2. TEMPORIZADOR para la máquina de estados
        self.create_timer(1.0, self.state_machine_step)
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
        self.get_logger().info("🌐 Vagando por el área.")
        # Ahora, la detección depende del suscriptor
        if self.current_gesture == Gesture.CALL_ROBOT:
            self.get_logger().info("✋ Comando 'CALL_ROBOT' (1) recibido → APPROACH")
            self.state = RobotState.APPROACH
            self.current_gesture = Gesture.NONE # Consumir el gesto

    def approach(self):
        self.get_logger().info("🚶 Acercándose al cliente. Transicionando a RECOGNIZE...")
        # Aquí iría la lógica de navegación
        self.state = RobotState.RECOGNIZE
        # No se consume el gesto aquí, ya que se asume que el robot llega y espera
        # El gesto se consume si se recibe en el estado RECOGNIZE.

    def recognize(self):
        self.get_logger().info("🤖 Esperando gesto del cliente. (2-6)")
        
        # Espera que el suscriptor haya actualizado self.current_gesture
        if self.current_gesture != Gesture.NONE:
            self.get_logger().info(f"✅ Gesto {self.current_gesture.name} reconocido.")
            self.state = RobotState.EXECUTE
        else:
            # Permanece en RECOGNIZE hasta que se presione una tecla
            pass

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