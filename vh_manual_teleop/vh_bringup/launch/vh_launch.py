import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Lança os nós do lado do veículo, ligados diretamente aos tópicos do carro (sem Autoware),
# e grava as métricas num bag.


def generate_launch_description():

    ip_address_arg = DeclareLaunchArgument(
        'ip_address', default_value='10.0.0.2',
        description="IP address of the remote station")
    ip_address = LaunchConfiguration('ip_address')

    input_port_arg = DeclareLaunchArgument(
        'input_port', default_value='5005',
        description="UDP port for input_teleop_decoder")
    input_port = LaunchConfiguration('input_port')

    telemetry_port_arg = DeclareLaunchArgument(
        'telemetry_port', default_value='5006',
        description="UDP port for telemetry_encoder")
    telemetry_port = LaunchConfiguration('telemetry_port')

    camera_port_arg = DeclareLaunchArgument(
        'camera_port', default_value='5007',
        description="Base UDP port for video, camera i uses camera_port + i")
    camera_port = LaunchConfiguration('camera_port')

    camera_config = PathJoinSubstitution([FindPackageShare('vh_bringup'), 'config', 'vehicle.yaml'])
    control_config = PathJoinSubstitution([FindPackageShare('vh_teleop_to_vehicle'), 'config', 'vehicle_control.yaml'])

    # rede
    input_teleop_decoder_node = Node(
        package='vh_network', executable='input_teleop_decoder', name='input_teleop_decoder',
        output='screen', parameters=[{'ip_address': ip_address, 'port': input_port}])

    telemetry_encoder_node = Node(
        package='vh_network', executable='telemetry_encoder', name='telemetry_encoder',
        output='screen', parameters=[{'ip_address': ip_address, 'port': telemetry_port}])

    video_encoder_node = Node(
        package='vh_network', executable='video_encoder', name='video_encoder',
        output='screen', parameters=[camera_config, {'ip_address': ip_address, 'port': camera_port}])

    # segurança da ligação
    topic_monitor_node = Node(
        package='vh_teleop_to_vehicle', executable='topic_monitor', name='topic_monitor',
        output='screen')

    safety_gate_node = Node(
        package='vh_teleop_to_vehicle', executable='safety_gate', name='safety_gate',
        output='screen')

    # carro
    vehicle_control_node = Node(
        package='vh_teleop_to_vehicle', executable='vehicle_control', name='vehicle_control',
        output='screen', parameters=[control_config])

    vehicle_telemetry_node = Node(
        package='vh_teleop_to_vehicle', executable='vehicle_telemetry', name='telemetry_node',
        output='screen')

    # bag com as métricas, uma pasta por execução
    bag_dir = os.path.expanduser('~/bags')
    os.makedirs(bag_dir, exist_ok=True)
    bag_commands_path = os.path.join(bag_dir, 'metrics', datetime.now().strftime('%Y%m%d_%H%M%S'))

    rosbag_metrics_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/metrics/network/teleop_commands',
            '/metrics/command_decoder',
            '/metrics/safety_gate',
            '/metrics/control',
            '/metrics/e2e_command_latency',
            '/metrics/telemetry_aggregator',
            '/teleop/telemetry',
            '/teleop/safety_state',
            '/teleop/vehicle_mode',
            '--output', bag_commands_path
        ],
        output='screen')

    return LaunchDescription([
        ip_address_arg,
        input_port_arg,
        telemetry_port_arg,
        camera_port_arg,
        input_teleop_decoder_node,
        telemetry_encoder_node,
        video_encoder_node,
        topic_monitor_node,
        safety_gate_node,
        vehicle_control_node,
        vehicle_telemetry_node,
        rosbag_metrics_node,
    ])
