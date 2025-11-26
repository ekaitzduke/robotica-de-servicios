import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mario/Documentos/Robotica de servicios/ros2_ws/install/robot_camarero'
