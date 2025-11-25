from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'manual_teleop'
    
    return LaunchDescription([
        # 1. Nó do Sistema: Leitura do Joystick (Logitech G923)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            # Parâmetros opcionais para o dispositivo (descomente se precisar)
            parameters=[{'deadzone': 0.001, 'autorepeat': 20.0}] 
        ),

        # 2. Nó de Mapeamento: Traduz Joystick para Comandos /teleop/
        Node(
            package=pkg_name,
            executable='teleop_g923_node',
            name='teleop_g923_node',
            output='screen',
        ),

        # 3. Nó de Controle: Recebe comandos e publica no /external/selected/control_cmd (10Hz)
        Node(
            package=pkg_name,
            executable='manual_teleop',
            name='manual_teleop',
            output='screen',
        ),
    ])
