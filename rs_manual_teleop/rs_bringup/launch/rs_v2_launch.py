from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
   
    # Argumento para o ID do dispositivo
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description="ID do dispositivo (Logitech ou Xbox) no sistema"
    )
    
    # Novo argumento: use_xbox (padrão é falso, ou seja, usa G923)
    use_xbox_arg = DeclareLaunchArgument(
        'use_xbox',
        default_value='false',
        description="Se true, utiliza o mapeamento para Xbox. Se false, usa G923."
    )

    video_mode_arg = DeclareLaunchArgument(
        'video_mode',
        default_value='none',
        description="Selecione o encoder de vídeo: 'none' (nenhum), 'standard' (normal), ou '4x'"
    )
    video_mode = LaunchConfiguration('video_mode')

    
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='10.0.0.1',
        description="Endereço IP para os nós de rede."
    )
    ip_address = LaunchConfiguration('ip_address')

    input_port_arg = DeclareLaunchArgument(
        'input_port', 
        default_value='5005', 
        description="Porta UDP para o input_teleop_encoder"
    )
    input_port = LaunchConfiguration('input_port')

    telemetry_port_arg = DeclareLaunchArgument(
        'telemetry_port', 
        default_value='5006', 
        description="Porta UDP para o telemetry_decoder"
    )
    telemetry_port = LaunchConfiguration('telemetry_port')

    camera_port_arg = DeclareLaunchArgument(
        'camera_port', 
        default_value='5007', 
        description="Porta base UDP para o(s) decoder(s) de vídeo"
    )
    camera_port = LaunchConfiguration('camera_port')


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

    input_teleop_encoder_node = Node(
        package="rs_network",
        executable='input_teleop_encoder_v2', 
        name='input_teleop_encoder_v2',
        output='screen',
        parameters=[{'ip_address': ip_address, 'port': input_port}]
    )

    telemetry_encoder_node = Node(
        package="rs_network",
        executable='telemetry_decoder', 
        name='telemetry_decoder',
        output='screen',
        parameters=[{'ip_address': ip_address, 'port': telemetry_port}]
    )

    # Nó opcional: video_encoder
    video_decoder_node = Node(
        package='vh_network',
        executable='video_decoder_v2',
        name='video_decoder_v2',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video_mode, "' == 'standard'"])),
        parameters=[{'ip_address': ip_address, 'port': camera_port}]
    )

    # Nó opcional: video_encoder_4x
    video_decoder_4x_node = Node(
        package='vh_network',
        executable='video_decoder_4x',
        name='video_decoder_4x',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video_mode, "' == '4x'"])),
        parameters=[{'ip_address': ip_address, 'port': camera_port}]
    )

    return LaunchDescription([
        device_id_arg,
        use_xbox_arg,
        video_mode_arg,
        ip_address_arg,
        input_port_arg,
        telemetry_port_arg,
        camera_port_arg,
        joy_node,
        g923_teleop_node,
        xbox_teleop_node,
        command_gate_node,
        input_teleop_encoder_node,
        telemetry_encoder_node,
        video_decoder_node,
        video_decoder_4x_node
    ])