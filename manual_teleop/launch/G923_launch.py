from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = 'manual_teleop'
    
    
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='ID of wheel+pedals'
    )
    
    device_id_config = LaunchConfiguration('device_id')
    
    # 1. Nó do Sistema: Leitura do Joystick (Logitech G923)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        # Parâmetros opcionais para o dispositivo (descomente se precisar)
        parameters=[
            {'deadzone': 0.001, 'autorepeat': 20.0},
            {'device_id': device_id_config}
        ] 
    )

    # 2. Nó de Mapeamento: Traduz Joystick para Comandos /teleop/
    teleop_node = Node(
        package=pkg_name,
        executable='teleop_g923_node',
        name='teleop_g923_node',
        output='screen',
    )

    # 3. Nó de Controle: Recebe comandos e publica no /external/selected/control_cmd (10Hz)
    manual_teleop_node = Node(
        package=pkg_name,
        executable='manual_teleop',
        name='manual_teleop',
        output='screen',
    )
    
    return LaunchDescription([
        device_id_arg,
        joy_node,
        teleop_node,
        manual_teleop_node
    ])
