from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Nó do Controlador Principal (Converte UDP para Autoware Control)
    control_node = Node(
        package='vh_teleop_to_autoware',
        executable='control',
        name='autoware_controller_node',
        output='screen'
    )

    # 2. Nó Monitor de Tópicos (Avalia a saúde da rede)
    monitor_node = Node(
        package='vh_teleop_to_autoware',
        executable='topic_monitor',
        name='teleop_topic_monitor_node',
        output='screen',
        parameters=[{
            'warn_timeout_ms': 100.0,  # 100ms sem dados = Aviso
            'error_timeout_ms': 250.0  # 250ms sem dados = Erro Crítico (Paragem)
        }]
    )

    # 3. Nó Safety Gate (Filtra os comandos baseados no estado da rede)
    safety_gate_node = Node(
        package='vh_teleop_to_autoware',
        executable='safety_gate',
        name='teleop_safety_gate_node',
        output='screen',
        parameters=[{
            'warning_velocity_limit': 2.77,   # ~10 km/h de limite em caso de aviso
            'emergency_deceleration': -3.0    # Travagem forte em caso de erro
        }]
    )

    return LaunchDescription([
        control_node,
        monitor_node,
        safety_gate_node
    ])