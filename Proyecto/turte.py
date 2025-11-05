import rclpy
from rclpy.node import Node
from enum import Enum

# Definimos los estados posibles
class RobotState(Enum):
    WANDER = 1
    APPROACH = 2
    RECOGNIZE = 3
    EXECUTE = 4

# Gestos posibles (simulados)
class Gesture(Enum):
    NONE = 0
    CALL_ROBOT = 1           # Mano levantada
    ORDER_COMPLETE = 2       # Pulgar arriba
    SELECT_ORDER = 3         # N dedos levantados
    ASK_FOR_BILL = 4         # Apuntar con dedos
    COMPLAINT_FORM = 5       # Pulgar abajo
    ASK_WIFI = 6             # Gesto teléfono

class WaiterRobot(Node):
    def _init_(self):
        super()._init_('waiter_robot')
        self.state = RobotState.WANDER
        self.current_gesture = Gesture.NONE

        # Aquí podrían ir tus suscriptores a cámara o detección de gestos
        self.create_timer(1.0, self.state_machine_step)
        self.get_logger().info("Robot camarero iniciado. Estado inicial: WANDER")

    def state_machine_step(self):
        """Ejecuta un ciclo de la máquina de estados."""
        if self.state == RobotState.WANDER:
            self.wander()

        elif self.state == RobotState.APPROACH:
            self.approach()

        elif self.state == RobotState.RECOGNIZE:
            self.recognize()

        elif self.state == RobotState.EXECUTE:
            self.execute()

    # === ESTADOS ===
    def wander(self):
        self.get_logger().info("🌐 Vagando por el área buscando una mano...")
        # Simular detección de gesto
        self.current_gesture = self.detect_gesture()

        if self.current_gesture == Gesture.CALL_ROBOT:
            self.get_logger().info("✋ Mano detectada → cambiando a APPROACH")
            self.state = RobotState.APPROACH

    def approach(self):
        self.get_logger().info("🚶 Acercándose al cliente...")
        # Aquí podrías incluir navegación o detección de posición
        self.state = RobotState.RECOGNIZE

    def recognize(self):
        self.get_logger().info("🤖 Reconociendo gesto del cliente...")
        gesture = self.detect_gesture()

        if gesture != Gesture.NONE:
            self.current_gesture = gesture
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

        # Vuelve al estado inicial
        self.state = RobotState.WANDER
        self.current_gesture = Gesture.NONE

    # === Simulación de detección de gestos ===
    def detect_gesture(self):
        """En un robot real, aquí se implementaría visión artificial."""
        # Por ahora simulamos entrada manual:
        # (En ROS real usarías suscriptores a un nodo de reconocimiento de gestos)
        import random
        gestures = list(Gesture)
        return random.choice(gestures)

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

if _name_ == '_main_':
    main()