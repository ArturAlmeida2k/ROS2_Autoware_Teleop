from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'interface'
    
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
            {'autorepeat_rate': 50.0},
            {'device_id': device_id_config}
        ] 
    )

    # 2. Nó de Mapeamento: Traduz Joystick para mensagens /teleop/
    teleop_node = Node(
        package=pkg_name,
        executable='g923_teleop_node',
        name='g923_teleop_node',
        output='screen'
    )

    return LaunchDescription([
        device_id_arg,
        joy_node,
        teleop_node,
    ])