#!/usr/bin/env python3

import time     # To delay state change as lack real functions

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

from yasmin.state import State
from yasmin.state_machine import StateMachine
from yasmin.blackboard import Blackboard
from yasmin_viewer import YasminViewerPub

import logging  # To disable yasmin messages when changing states



# Wander State: Robot wanders searching for an available target (then goes to chase)
# (Used a timer to simulate detection. Substitute with a subscription to camera and termination based on detection instead of steps)
class WanderState(State):
    def __init__(self,node):
        super().__init__(outcomes={'chase'})

        self._node = node

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self._node)

        self._timer = self._node.create_timer(0.1, self.tick)
        self._timer.cancel()

        self._pub_vel = None

        self.max_steps = 20
        self.steps = 0

        self.end = False

    def execute(self, blackboard: Blackboard) -> str:
        print(f"→ Wandering\n")
        blackboard.mensaje = f"Starting wander"

        if self._pub_vel is None:
            self._pub_vel = self._node.create_publisher(Twist, '/cmd_vel', 10)

        self.steps = 0
        self.end = False

        self._timer.reset()
        while rclpy.ok() and not self.end:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._timer.cancel()

        return 'chase'
    

    def tick(self):

        self.end = not (self.steps < self.max_steps)
        cmd = Twist()

        if not self.end:
            cmd.linear.x = 0.1  # Avanzar más rápido
            cmd.angular.z = 0.4 # Sin girar
            self.steps += 1
        else:
            # Cuando termina la distancia, frenamos y cambiamos de estado
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0


        if self._pub_vel:
            self._pub_vel.publish(cmd)



# Chase State: Robot finds a target and starts approaching it (can catch it or lose it)
class ChaseState(State):
    def __init__(self):
        super().__init__(outcomes={'lost','catch'})
        self.found = False

    def execute(self, blackboard: Blackboard) -> str:
        print("→ Chasing")
        time.sleep(2.0)
        if not self.found:
            print("  Chase → Lost")
            self.found = True
            return 'lost'
        
        self.found = False
        print("  Chase → Catch")
        return 'catch'


# Lost State: Robot has lost object recently and it is searching for it 
# (Failed searchs make robot wander again. Successful ones reinitiate the chase)
class LostState(State):
    def __init__(self):
        super().__init__(outcomes={'wander','chase'})
        self.lucky = False

    def execute(self, blackboard: Blackboard) -> str:
        print("→ Object Lost")
        time.sleep(2.0)
        if not self.lucky:
            print("  Lost → Wandering")
            self.lucky = True
            return 'wander'
        
        self.lucky = False
        print("  Lost → Chase")
        return 'chase'

# Catch State: Robot has caught the target after completing the chase
# (It will erase the target for the list and check if more targets are needed to be caught to go back wandering)
# (In case all targets are caught, task and node terminates)
class CatchState(State):
    def __init__(self):
        super().__init__(outcomes={'wander','finished'})
        self.count = 0

    def execute(self, blackboard: Blackboard) -> str:
        print("→ Object Cought")
        time.sleep(2.0)
        self.count += 1
        if self.count < 3:
            print("  Cought → Wandering")
            print(f"  Total objects cought: {self.count}")
            return 'wander'

        return 'finished'


def main():

    logging.disable(logging.INFO)  # Disable YASMIN message (but also any other message)

    rclpy.init()

    # Basic definition of terminate state of FSM and the states
    sm = StateMachine(outcomes={'finished'})
    node = Node('chaser_robot')
    wandering = WanderState(node)
    chasing = ChaseState()
    object_lost = LostState()
    object_cought = CatchState()

    # Register of states and transitions on FSM
    sm.add_state('wandering', wandering, transitions={'chase': 'chasing'})
    sm.add_state('chasing', chasing, transitions={'lost': 'oblost', 'catch': 'obcought'})
    sm.add_state('oblost', object_lost, transitions={'wander': 'wandering', 'chase': 'chasing'})
    sm.add_state('obcought', object_cought, transitions={'wander': 'wandering', 'finished': 'finished'})
    sm.set_start_state('wandering')

    sm.validate()  # Optional check of the FSM

    # Start publishing on yasmin_viewer and execute the FSM
    YasminViewerPub("ROBOT CHASER YASMIN", sm)
    outcome = sm.execute(Blackboard())
    print(f"FSM ends with outcome: {outcome}")

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
