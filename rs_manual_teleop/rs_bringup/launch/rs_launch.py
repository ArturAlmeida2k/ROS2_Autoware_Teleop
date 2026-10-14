import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from datetime import datetime

# Lança a estação remota: joystick, mapeamento do controlador, rede, GUI e bag de métricas.

def generate_launch_description():

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description="Joystick device ID (Logitech or Xbox)"
    )
    device_id_config = LaunchConfiguration('device_id')

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='rs50',
        description="Controller type: 'rs50' (default), 'g923' or 'xbox'"
    )
    controller = LaunchConfiguration('controller')

    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='10.0.0.1',
        description="IP address used by the network nodes"
    )
    ip_address = LaunchConfiguration('ip_address')

    input_port_arg = DeclareLaunchArgument(
        'input_port',
        default_value='5005',
        description="UDP port for input_teleop_encoder"
    )
    input_port = LaunchConfiguration('input_port')

    telemetry_port_arg = DeclareLaunchArgument(
        'telemetry_port',
        default_value='5006',
        description="UDP port for telemetry_decoder"
    )
    telemetry_port = LaunchConfiguration('telemetry_port')

    pointcloud_port_arg = DeclareLaunchArgument(
        'pointcloud_port',
        default_value='5011',
        description="TCP port for pointcloud_decoder"
    )
    pointcloud_port = LaunchConfiguration('pointcloud_port')

    max_vlc_arg = DeclareLaunchArgument(
        'max_vlc',
        default_value='30.0',
        description="Max velocity in km/h at full throttle"
    )
    max_vlc = LaunchConfiguration('max_vlc')

    # joy limitado a 60 Hz pelo throttle
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[
            {'deadzone': 0.001},
            {'autorepeat_rate': 100.0},
            {'device_id': device_id_config}
        ]
    )

    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='joy_throttle',
        arguments=['messages', '/joy', '60.0', '/joy_throttled']
    )

    # só arranca o nó do controlador escolhido
    rs50_teleop_node = Node(
        package="rs_interface",
        executable='rs50_teleop_node',
        name='rs50_teleop_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'rs50'"]))
    )

    g923_teleop_node = Node(
        package="rs_interface",
        executable='g923_teleop_node',
        name='g923_teleop_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'g923'"]))
    )

    xbox_teleop_node = Node(
        package="rs_interface",
        executable='xbox_teleop_node',
        name='xbox_teleop_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'xbox'"]))
    )

    command_gate_node = Node(
        package="rs_interface",
        executable='command_gate',
        name='command_gate',
        output='screen',
        parameters=[{'max_vlc': ParameterValue(max_vlc, value_type=float)}]
    )

    input_teleop_encoder_node = Node(
        package="rs_network",
        executable='input_teleop_encoder',
        name='input_teleop_encoder',
        output='screen',
        parameters=[{'ip_address': ip_address, 'port': input_port}]
    )

    telemetry_decoder_node = Node(
        package="rs_network",
        executable='telemetry_decoder',
        name='telemetry_decoder',
        output='screen',
        parameters=[{'ip_address': ip_address, 'port': telemetry_port}]
    )

    pointcloud_decoder_node = Node(
        package="rs_network",
        executable='pointcloud_decoder',
        name='pointcloud_decoder',
        output='screen',
        parameters=[{'ip_address': ip_address, 'port': pointcloud_port}]
    )

    gui_node = Node(
        package="rs_gui",
        executable='rs_gui',
        name='rs_gui',
        output='screen'
    )

    # bag com as métricas, uma pasta por execução
    bag_dir = os.path.expanduser('~/bags')
    os.makedirs(bag_dir, exist_ok=True)

    bag_metrics_path = os.path.join(bag_dir, f'metrics/{datetime.now().strftime("%Y%m%d_%H%M%S")}')

    rosbag_metrics_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/metrics/controller',
            '/metrics/command_gate',
            '/metrics/network/telemetry',
            '/metrics/telemetry_decoder',
            '/metrics/telemetry_gui',
            '/metrics/e2e_telemetry_latency',
            '/metrics/full_latency',
            '/metrics/front_camera',
            '/metrics/front_camera_network',
            '/metrics/front_camera_decode',
            '/metrics/front_camera_encode',
            '/metrics/network/pointcloud',
            '/metrics/e2e_pointcloud_latency',
            '/metrics/pointcloud_size',
            '/metrics/pointcloud_gui',
            '--output', bag_metrics_path
        ],
        output='screen'
    )

    return LaunchDescription([
        device_id_arg,
        controller_arg,
        ip_address_arg,
        input_port_arg,
        telemetry_port_arg,
        pointcloud_port_arg,
        max_vlc_arg,
        joy_node,
        throttle_node,
        rs50_teleop_node,
        g923_teleop_node,
        xbox_teleop_node,
        command_gate_node,
        input_teleop_encoder_node,
        telemetry_decoder_node,
        pointcloud_decoder_node,
        gui_node,
        rosbag_metrics_node
    ])