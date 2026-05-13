from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
   
    # Argumento para o ID do dispositivo
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description='ID do dispositivo (Logitech ou Xbox) no sistema'
    )
    
    # Novo argumento: use_xbox (padrão é falso, ou seja, usa G923)
    use_xbox_arg = DeclareLaunchArgument(
        'use_xbox',
        default_value='false',
        description='Se "true", utiliza o mapeamento para Xbox. Se "false", usa G923.'
    )
    
    device_id_config = LaunchConfiguration('device_id')
    use_xbox_config = LaunchConfiguration('use_xbox')
    
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

    # 2a. Nó de Mapeamento: Logitech G923 (Ativo se use_xbox for false)
    g923_teleop_node = Node(
        package="rs_interface",
        executable='g923_teleop_node_v2',
        name='g923_teleop_node_v2',
        output='screen',
        condition=UnlessCondition(use_xbox_config)
    )

    # 2b. Nó de Mapeamento: Xbox (Ativo se use_xbox for true)
    xbox_teleop_node = Node(
        package="rs_interface",
        executable='xbox_teleop_node_v2',
        name='xbox_teleop_node_v2',
        output='screen',
        condition=IfCondition(use_xbox_config)
    )

    # 3. Gate e Network
    command_gate_node = Node(
        package="rs_interface",
        executable='command_gate',
        name='command_gate',
        output='screen'
    )

    encoder_node = Node(
        package="rs_network",
        executable='input_teleop_encoder_v2', 
        name='input_teleop_encoder_v2',
        output='screen'
    )

    return LaunchDescription([
        device_id_arg,
        use_xbox_arg,
        joy_node,
        g923_teleop_node,
        xbox_teleop_node,
        command_gate_node,
        encoder_node
    ])