import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # --- Argumentos do Launch ---
    num_cameras_arg = DeclareLaunchArgument(
        'num_cameras',
        default_value='0',
        description="Número de câmaras a transmitir (0 desliga o encoder de vídeo, 1..4)"
    )
    num_cameras = LaunchConfiguration('num_cameras')

    sim_arg = DeclareLaunchArgument(
        'sim', default_value='awsim',
        description="Perfil de configuração: 'awsim' ou 'carla'")
    sim = LaunchConfiguration('sim')
    
    config_file = PathJoinSubstitution([
        get_package_share_directory('vh_bringup'), 'config',
        PythonExpression(["'", sim, "' + '.yaml'"])
    ])


    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='10.0.0.2',
        description="Endereço IP da estação de controlo"
    )
    ip_address = LaunchConfiguration('ip_address')

    # --- Portas ---
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

    camera_port_arg = DeclareLaunchArgument(
        'camera_port',
        default_value='5007',
        description="Porta base UDP do vídeo. A câmara i usa camera_port + i"
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

    # Encoder de vídeo unificado: 1 a 4 câmaras, uma porta por câmara.
    # A ordem dos tópicos define a ordem das portas e tem de corresponder
    # à que a GUI espera: base+0 front, base+1 left, base+2 back, base+3 right.
    video_encoder_node = Node(
        package='vh_network',
        executable='video_encoder',
        name='video_encoder',
        output='screen',
        condition=IfCondition(PythonExpression([num_cameras, " > 0"])),
        parameters=[
            config_file,
            {'ip_address': ip_address, 'port': camera_port},
        ]
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

    # --- Rosbag ---
    bag_dir = os.path.expanduser('~/bags')
    os.makedirs(bag_dir, exist_ok=True)

    bag_commands_path = os.path.join(
        bag_dir, 'metrics', datetime.now().strftime('%Y%m%d_%H%M%S'))

    rosbag_metrics_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/metrics/network/teleop_commands',
            '/metrics/command_decoder',
            '/metrics/safety_gate',
            '/metrics/control',
            '/metrics/e2e_command_latency',
            '/metrics/telemetry_aggregator',
            '--output', bag_commands_path
        ],
        output='screen'
    )

    return LaunchDescription([
        num_cameras_arg,
        sim_arg,
        ip_address_arg,
        input_port_arg,
        telemetry_port_arg,
        camera_port_arg,
        input_teleop_decoder_node,
        telemetry_encoder_node,
        video_encoder_node,
        telemetry_node,
        control_node,
        safety_gate_node,
        topic_monitor_node,
        rosbag_metrics_node,
    ])