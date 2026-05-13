import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- Argumentos do Launch ---
    # Permite escolher qual encoder de vídeo rodar: 'none', 'standard', ou '4x'
    video_mode_arg = DeclareLaunchArgument(
        'video_mode',
        default_value='none',
        description="Selecione o encoder de vídeo: 'none' (nenhum), 'standard' (normal), ou '4x'"
    )
    video_mode = LaunchConfiguration('video_mode')

    # --- Nós do pacote: vh_network ---
    input_teleop_decoder_node = Node(
        package='vh_network',
        executable='input_teleop_decoder',
        name='input_teleop_decoder',
        output='screen'
    )

    telemetry_encoder_node = Node(
        package='vh_network',
        executable='telemetry_encoder',
        name='telemetry_encoder',
        output='screen'
    )

    # Nó opcional: video_encoder
    video_encoder_node = Node(
        package='vh_network',
        executable='video_encoder',
        name='video_encoder',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video_mode, "' == 'standard'"]))
    )

    # Nó opcional: video_encoder_4x
    video_encoder_4x_node = Node(
        package='vh_network',
        executable='video_encoder_4x',
        name='video_encoder_4x',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video_mode, "' == '4x'"]))
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
        package='vh_teleop_to_autoware',
        executable='control',
        name='control',
        output='screen'
    )

    teleop_safety_gate_node = Node(
        package='vh_teleop_to_autoware',
        executable='teleop_safety_gate',
        name='teleop_safety_gate',
        output='screen'
    )

    teleop_topic_monitor_node = Node(
        package='vh_teleop_to_autoware',
        executable='teleop_topic_monitor',
        name='teleop_topic_monitor',
        output='screen'
    )

    return LaunchDescription([
        video_mode_arg,
        input_teleop_decoder_node,
        telemetry_encoder_node,
        video_encoder_node,
        video_encoder_4x_node,
        telemetry_node,
        control_node,
        teleop_safety_gate_node,
        teleop_topic_monitor_node
    ])