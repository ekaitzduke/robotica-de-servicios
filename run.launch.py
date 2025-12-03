import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. CONFIGURACIÓN ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Usar tiempo sim')

    # --- 2. RUTAS ---
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    tb4_gz_bringup_share = get_package_share_directory('turtlebot4_gz_bringup')

    # --- 3. LANZAR GAZEBO (EL MUNDO) ---
    # Aquí le decimos que cargue el ARCHIVO 'empty.sdf'
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items() # -r para que arranque solo
    )

    # --- 4. SPAWNEAR EL ROBOT ---
    # Usamos 'turtlebot4_spawn.launch.py' que vimos en tu 'ls'
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_gz_bringup_share, 'launch', 'turtlebot4_spawn.launch.py')
        ),
        launch_arguments={
            'model': 'standard',
            'use_sim_time': use_sim_time,
            # IMPORTANTE: Coordenadas para que no caiga al vacío si el suelo tarda en cargar
            'x': '0.0', 'y': '0.0', 'z': '0.1' 
        }.items()
    )

    # --- 5. LANZAR NODOS Y PUENTES ---
    # Usamos 'turtlebot4_nodes.launch.py'
    # AQUÍ ESTÁ EL TRUCO: Le pasamos 'world' SIN EXTENSIÓN (.sdf)
    bringup_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_gz_bringup_share, 'launch', 'turtlebot4_nodes.launch.py')
        ),
        launch_arguments={
            'model': 'standard',
            'use_sim_time': use_sim_time,
            'world': 'empty' # <--- ¡SIN EL .sdf PARA EVITAR EL ERROR!
        }.items()
    )

    # --- 6. CREAR PUENTE DEL RELOJ ---
    # Es VITAL para que el controller_manager (y por tanto el diffdrive_controller)
    # se sincronicen con el tiempo de la simulación.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # --- 6. LANZAR EL NODO TURTE ---
    turte_node = Node(
        package='serv_proy',     # Nombre de tu paquete
        executable='turte',           # CORRECCIÓN: El ejecutable se llama como el script .py
        name='turte_node',            # Nombre que tendrá en el grafo de ROS
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # --- 7. DEVOLVER TODO ---
    return LaunchDescription([
        declare_use_sim_time,
        gz_sim,
        clock_bridge,
        spawn_robot,
        bringup_nodes,
        turte_node
    ])