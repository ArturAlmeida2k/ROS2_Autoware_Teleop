from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'manual_teleop'
    
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='ID do volante Logitech no PC B'
    )
    
    device_id_config = LaunchConfiguration('device_id')
    
    # 1. Nó do Sistema: Leitura do Joystick
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'deadzone': 0.001},
            {'autorepeat_rate': 20.0},
            {'device_id': device_id_config}
        ] 
    )

    # 2. Nó de Mapeamento: Traduz Joystick para mensagens /teleop/
    teleop_node = Node(
        package=pkg_name,
        executable='teleop_g923_node',
        name='teleop_g923_node',
        output='screen'
    )

    # 3. Nó Encoder (Gateway): Envia os tópicos via UDP para o PC A
    encoder_node = Node(
        package=pkg_name,
        executable='PCB_encoder', 
        name='PCB_encoder',
        output='screen'
    )

    return LaunchDescription([
        device_id_arg,
        joy_node,
        teleop_node,
        encoder_node
    ])