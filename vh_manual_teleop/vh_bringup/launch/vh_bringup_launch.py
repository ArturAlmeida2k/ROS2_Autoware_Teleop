import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- Argumentos Gerais ---
    video_mode_arg = DeclareLaunchArgument(
        'video_mode', default_value='none', description="Encoder: 'none', 'standard', ou '4x'"
    )
    video_mode = LaunchConfiguration('video_mode')

    server_ip_arg = DeclareLaunchArgument(
        'server_ip', default_value='10.0.0.2', description="Endereço IP para os nós de rede"
    )
    server_ip = LaunchConfiguration('server_ip')

    # --- Argumentos para as Portas (Sockets) ---
    input_port_arg = DeclareLaunchArgument(
        'input_port', default_value='5005', description="Porta UDP para o input_teleop_decoder"
    )
    input_port = LaunchConfiguration('input_port')

    telemetry_port_arg = DeclareLaunchArgument(
        'telemetry_port', default_value='5006', description="Porta UDP para o telemetry_encoder"
    )
    telemetry_port = LaunchConfiguration('telemetry_port')

    camera_port_arg = DeclareLaunchArgument(
        'camera_port', default_value='5007', description="Porta base (5007+) para o vídeo"
    )
    camera_port = LaunchConfiguration('camera_port')


    # --- Nós do pacote: vh_network ---
    input_teleop_decoder_node = Node(
        package='vh_network',
        executable='input_teleop_decoder',
        name='input_teleop_decoder',
        output='screen',
        parameters=[{
            'server_ip': server_ip,
            'port': input_port      
        }]
    )

    telemetry_encoder_node = Node(
        package='vh_network',
        executable='telemetry_encoder',
        name='telemetry_encoder',
        output='screen',
        parameters=[{
            'server_ip': server_ip,
            'port': telemetry_port   
        }]
    )

    video_encoder_node = Node(
        package='vh_network',
        executable='video_encoder',
        name='video_encoder',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video_mode, "' == 'standard'"])),
        parameters=[{
            'server_ip': server_ip,
            'port': camera_port      
        }]
    )

    video_encoder_4x_node = Node(
        package='vh_network',
        executable='video_encoder_4x',
        name='video_encoder_4x',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video_mode, "' == '4x'"])),
        parameters=[{
            'server_ip': server_ip,
            'port': camera_port    
        }]
    )

    # --- Nós do pacote: vh_telemetry ---
    telemetry_node = Node(
        package='vh_telemetry',
        executable='telemetry_node',
        name='telemetry_node',
        output='screen'
    )

    # --- Nós do pacote: teleop_to_autoware ---
    control_node = Node(
        package='teleop_to_autoware',
        executable='control',
        name='control',
        output='screen'
    )

    teleop_safety_gate_node = Node(
        package='teleop_to_autoware',
        executable='teleop_safety_gate',
        name='teleop_safety_gate',
        output='screen'
    )

    teleop_topic_monitor_node = Node(
        package='teleop_to_autoware',
        executable='teleop_topic_monitor',
        name='teleop_topic_monitor',
        output='screen'
    )

    return LaunchDescription([
        video_mode_arg,
        server_ip_arg,
        input_port_arg,     
        telemetry_port_arg,  
        camera_port_arg,     
        input_teleop_decoder_node,
        telemetry_encoder_node,
        video_encoder_node,
        video_encoder_4x_node,
        telemetry_node,
        control_node,
        teleop_safety_gate_node,
        teleop_topic_monitor_node
    ])