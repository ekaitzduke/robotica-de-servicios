import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- 6. LANZAR EL NODO TURTE ---
    turte_node = Node(
        package='robot_camarero',     # Nombre de tu paquete
        executable='turte_real',           # CORRECCIÓN: El ejecutable se llama como el script .py
        name='turte_node',            # Nombre que tendrá en el grafo de ROS
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # --- 7. DEVOLVER TODO ---
    return LaunchDescription([
        turte_node
    ])