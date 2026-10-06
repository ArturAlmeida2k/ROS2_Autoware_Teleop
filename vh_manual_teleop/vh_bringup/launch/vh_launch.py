import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node

# Lança os nós do lado do veículo (rede, telemetria, controlo) e grava as métricas num bag.


def generate_launch_description():

    sim_arg = DeclareLaunchArgument(
        'sim', default_value='awsim',
        description="Config profile: 'awsim' or 'carla'")
    sim = LaunchConfiguration('sim')

    config_file = PathJoinSubstitution([
        get_package_share_directory('vh_bringup'), 'config',
        PythonExpression(["'", sim, "' + '.yaml'"])
    ])

    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='10.0.0.2',
        description="IP address of the remote station"
    )
    ip_address = LaunchConfiguration('ip_address')

    input_port_arg = DeclareLaunchArgument(
        'input_port',
        default_value='5005',
        description="UDP port for input_teleop_decoder"
    )
    input_port = LaunchConfiguration('input_port')

    telemetry_port_arg = DeclareLaunchArgument(
        'telemetry_port',
        default_value='5006',
        description="UDP port for telemetry_encoder"
    )
    telemetry_port = LaunchConfiguration('telemetry_port')

    camera_port_arg = DeclareLaunchArgument(
        'camera_port',
        default_value='5007',
        description="Base UDP port for video, camera i uses camera_port + i"
    )
    camera_port = LaunchConfiguration('camera_port')

    pointcloud_port_arg = DeclareLaunchArgument(
        'pointcloud_port',
        default_value='5011',
        description="TCP port for pointcloud_encoder"
    )
    pointcloud_port = LaunchConfiguration('pointcloud_port')

    # vh_network
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

    video_encoder_node = Node(
        package='vh_network',
        executable='video_encoder',
        name='video_encoder',
        output='screen',
        parameters=[
            config_file,
            {'ip_address': ip_address, 'port': camera_port},
        ]
    )

    pointcloud_encoder_node = Node(
        package='vh_network',
        executable='pointcloud_encoder',
        name='pointcloud_encoder',
        output='screen',
        parameters=[{'ip_address': ip_address, 'port': pointcloud_port}]
    )

    telemetry_node = Node(
        package='vh_telemetry',
        executable='telemetry_node',
        name='telemetry_node',
        output='screen'
    )

    # vh_teleop_to_autoware
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

    # bag com as métricas, uma pasta por execução
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
            '/metrics/video_encoder/preprocess',
            '/metrics/video_encoder/x264',
            '/metrics/video_encoder/total',
            '--output', bag_commands_path
        ],
        output='screen'
    )

    return LaunchDescription([
        sim_arg,
        ip_address_arg,
        input_port_arg,
        telemetry_port_arg,
        camera_port_arg,
        pointcloud_port_arg,
        input_teleop_decoder_node,
        telemetry_encoder_node,
        video_encoder_node,
        pointcloud_encoder_node,
        telemetry_node,
        control_node,
        safety_gate_node,
        topic_monitor_node,
        rosbag_metrics_node,
    ])