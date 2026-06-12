import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from datetime import datetime


def generate_launch_description():
    
    # --- Argumentos do Launch ---
    video_mode_arg = DeclareLaunchArgument(
        'video_mode',
        default_value='none',
        description="Selecione o encoder de vídeo: 'none' (nenhum), 'standard' (normal), ou '4x'"
    )
    video_mode = LaunchConfiguration('video_mode')

    ip_address_arg = DeclareLaunchArgument(
        'ip_address', 
        default_value='10.0.0.2', 
        description="Endereço IP para os nós de rede"
    )
    ip_address = LaunchConfiguration('ip_address')

    # --- Argumentos das Portas ---
    input_port_arg = DeclareLaunchArgument(
        'input_port', 
        default_value='5005', 
        description="Porta UDP para o input_teleop_decoder"
    )
    input_port = LaunchConfiguration('input_port')

    telemetry_port_arg = DeclareLaunchArgument(
        'telemetry_port', 
        default_value='5006', 
        description="Porta UDP para o telemetry_encoder"
    )
    telemetry_port = LaunchConfiguration('telemetry_port')

    rtt_port_arg = DeclareLaunchArgument(
        'rtt_port', 
        default_value='5011', 
        description="Porta UDP para o rtt_metrics"
    )
    rtt_port = LaunchConfiguration('rtt_port')


    camera_port_arg = DeclareLaunchArgument(
        'camera_port', 
        default_value='5007', 
        description="Porta base UDP para o(s) encoder(s) de vídeo"
    )
    camera_port = LaunchConfiguration('camera_port')

    # --- Nós do pacote: vh_network ---
    input_teleop_decoder_node = Node(
        package='vh_network',
        executable='input_teleop_decoder',
        name='input_teleop_decoder',
        output='screen',
        parameters=[{'ip_address': ip_address, 'port': input_port}]
    )

    telemetry_encoder_node = Node(
        package='vh_network',
        executable='telemetry_encoder',
        name='telemetry_encoder',
        output='screen',
        parameters=[{'ip_address': ip_address, 'port': telemetry_port}]
    )

    rtt_metrics_node = Node(
        package='vh_network',
        executable='rtt_metrics',
        name='rtt_metrics',
        output='screen',
        parameters=[{'ip_address': ip_address, 'port': rtt_port}]
    )

    # Nó opcional: video_encoder
    video_encoder_node = Node(
        package='vh_network',
        executable='video_encoder',
        name='video_encoder',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video_mode, "' == 'standard'"])),
        parameters=[{'ip_address': ip_address, 'port': camera_port}]
    )

    # Nó opcional: video_encoder_4x
    video_encoder_4x_node = Node(
        package='vh_network',
        executable='video_encoder_4x',
        name='video_encoder_4x',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video_mode, "' == '4x'"])),
        parameters=[{'ip_address': ip_address, 'port': camera_port}]
    )

    # --- Nós do pacote: vh_telemetry ---
    telemetry_node = Node(
        package='vh_telemetry',
        executable='telemetry_node',
        name='telemetry_node',
        output='screen'
    )

    # --- Nós do pacote: vh_teleop_to_autoware ---
    control_node = Node(
        package='vh_teleop_to_autoware',
        executable='control',
        name='control',
        output='screen'
    )

    safety_gate_node = Node(
        package='vh_teleop_to_autoware',
        executable='safety_gate',
        name='safety_gate',
        output='screen'
    )

    topic_monitor_node = Node(
        package='vh_teleop_to_autoware',
        executable='topic_monitor',
        name='topic_monitor',
        output='screen'
    )

    # Rosbag
    bag_dir = os.path.expanduser('~/bags')
    os.makedirs(bag_dir, exist_ok=True)  
    
    bag_commands_path = os.path.join(bag_dir, f'commands/{datetime.now().strftime("%Y%m%d_%H%M%S")}')

    rosbag_commands_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/metrics/teleop_commands',
            '--output', bag_commands_path
        ],
        output='screen'
    )
    
    return LaunchDescription([
        video_mode_arg,
        ip_address_arg,
        input_port_arg,
        telemetry_port_arg,
        rtt_port_arg,
        camera_port_arg,
        input_teleop_decoder_node,
        telemetry_encoder_node,
        video_encoder_node,
        video_encoder_4x_node,
        telemetry_node,
        rtt_metrics_node,
        control_node,
        safety_gate_node,
        topic_monitor_node,
        rosbag_commands_node
    ])