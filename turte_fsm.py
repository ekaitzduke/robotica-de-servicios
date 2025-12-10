import rclpy
from rclpy.node import Node
from enum import Enum

import time

from std_msgs.msg import Int8 # Usaremos Int8 para mandar números desde el teclado
from geometry_msgs.msg import Twist

from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_viewer import YasminViewerPub

from tf2_ros import Buffer, TransformListener

import logging  # To disable yasmin messages when changing states

# Wander State: Robot wanders searching for customers gestures (then goes to approach)
class WanderState(State):
    def __init__(self, node):
        super().__init__(outcomes={'approach'})
        self._node =  node
        self._pub_vel = None
        self._sub_gesture = None
        self.current_gesture = 0

        self._node.declare_parameter('current_gesture',rclpy.Parameter.Type.INTEGER)
        self._node.set_parameters([rclpy.parameter.Parameter('current_gesture',rclpy.Parameter.Type.INTEGER,0)])

    def execute(self, blackboard: Blackboard) -> str:
        print(f"→ Wandering")

        self._sub_gesture = self.create_subscription(Int8, '/comando_gesto', self.gesture_callback, 10)
        self._pub_vel = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        while rclpy.ok() and self.current_gesture is not 1:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        if self._sub_gesture:
            self._node.destroy_subscription(self._sub_gesture)
            self._sub_gesture = None

        self.get_logger().info("✋ 'CALL_ROBOT' recibido -> Acercándose...")
        self.current_gesture = 0
        self._node.set_parameters([rclpy.parameter.Parameter('current_gesture',rclpy.Parameter.Type.INTEGER,0)])
        print("  Wandering → Approaching")
        return 'approach'

    def gesture_callback(self, msg):
        """Función que se activa al recibir un comando por teclado."""
        # Convierte el número recibido (msg.data) al Enum Gesture correspondiente
        self.current_gesture = msg.data
        self.get_logger().info(f"⌨️ Comando recibido: {msg.data}")

        cmd = Twist()
        cmd.linear.x = 0.3   # Velocidad hacia adelante
        cmd.angular.z = 0.5  # Velocidad de giro

        if self._pub_vel:
            self._pub_vel.publish(cmd)




# Approach State: Robot approaches the customer (then goes to recognize)
class ApproachState(State):
    def __init__(self, node):
        super().__init__(outcomes={'approach'})
        self._node =  node

        self._pub_vel = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.create_timer(1, self.tick)

        self.reached = False
        self.steps_counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        print(f"→ Approaching")

        self._pub_vel = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)

        while rclpy.ok() and not self.reached:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self.get_logger().info("🛑 Llegada al cliente. Esperando orden (RECOGNIZE).")
        print("  Approaching → Recognize")
        return 'recognize'

    def tick(self):
        # LÓGICA: Avanzar recto una distancia corta (durante 40 ciclos = 4 segundos)
        limit_steps = 40

        self.reached = not (self.steps_counter < limit_steps)
        cmd = Twist()

        if not self.reached:
            cmd.linear.x = 0.5  # Avanzar más rápido
            cmd.angular.z = 0.0 # Sin girar
            self.steps_counter += 1
        else:
            # Cuando termina la distancia, frenamos y cambiamos de estado
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.steps_counter = 0


        if self._pub_vel:
            self._pub_vel.publish(cmd)




# Recognize State: Robot tries to recognize the gesture in front of it (Then execute action)
class RecognizeState(State):
    def __init__(self, node):
        super().__init__(outcomes={'task'})
        self._node =  node

        self._pub_vel = None
        self._sub_gesture = None
        self.current_gesture = 0


    def execute(self, blackboard: Blackboard) -> str:
        print(f"→ Recognize")

        self._pub_vel = self.create_publisher(Twist, '/cmd_vel_unstamped', 10)
        self._sub_gesture = self.create_subscription(Int8, '/comando_gesto', self.gesture_callback, 10)

        cmd = Twist()
        cmd.linear.x = 0.0   # Velocidad hacia adelante
        cmd.angular.z = 0.0  # Velocidad de giro

        if self._pub_vel:
            self._pub_vel.publish(cmd)
            self._node.destroy_subscription(self._sub_gesture)
            self._sub_gesture = None

        while rclpy.ok() and self.current_gesture < 1:
            rclpy.spin_once(self._node, timeout_sec=0.1)

        self.get_logger().info(f"✅ Gesto {self.current_gesture} entendido.")

        print("  Recognize → Task")
        return 'task'
    
    def gesture_callback(self, msg):
        """Función que se activa al recibir un comando por teclado."""
        # Convierte el número recibido (msg.data) al Enum Gesture correspondiente
        self.current_gesture = msg.data
        self.get_logger().info(f"⌨️ Comando recibido: {msg.data}")
        self._node.set_parameters([rclpy.parameter.Parameter('current_gesture',rclpy.Parameter.Type.INTEGER,msg.data)])





# Task State: Robot executes the given task by the customer
class TaskState(State):
    def __init__(self, node):
        super().__init__(outcomes={'wander','recognize'})
        self._node =  node


    def execute(self, blackboard: Blackboard) -> str:
        current_gesture = self._node.get_parameter('current_gesture').value
        print(f"→ Task")

        if current_gesture == 2:
            self.get_logger().info("✅ Pedido completado, regresando a WANDER.")
            return 'wander'
        elif current_gesture == 3:
            self.get_logger().info("💳 Cliente pidió la cuenta.")
        elif current_gesture == 4:
            self.get_logger().info("📄 Entregando hoja de reclamaciones.")
        elif current_gesture > 4 and self.current_gesture < 9:
            self.get_logger().info("🍽️ Pedido recibido, ejecutando...")
        else:
            # Esto maneja el caso de que se haya recibido un 1 (CALL_ROBOT) mientras ya estaba aquí.
            self.get_logger().info("❓ Gesto ambiguo o no ejecutable. Haga un gesto correcto.")
            
        print("  Task → Recognize")
        return 'recognize'




def main(args=None):
    rclpy.init(args=args)
    node = Node('waiter_robot')
    wanderState = WanderState(node)
    approachState = ApproachState(node)
    recognizeState = RecognizeState(node)
    taskState = TaskState(node)

    # FSM YASMIN
    sm = StateMachine(outcomes={'end'})
    estados_cancelables = [wanderState, approachState,recognizeState,taskState] 
    sm.add_state('wandering', wanderState,
                 transitions={'approach': 'approaching'})
    sm.add_state('approaching', approachState,
                 transitions={'recognize':'recognizing'})
    sm.add_state('recognizing', recognizeState,
                 transitions={'task':'taskjob'})
    sm.add_state('taskjob', taskState,
                 transitions={'recognize':'recognizing', 'wander': 'wandering'})
    sm.set_start_state('wandering')
    sm.validate()

    YasminViewerPub("WAITER ROBOT YASMIN", sm)

    try:
        outcome = sm.execute(Blackboard())
        node.get_logger().info(f"FSM ended with outcome: {outcome}")
    except KeyboardInterrupt:
        for state in estados_cancelables: 
            state.cancel_state()
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()