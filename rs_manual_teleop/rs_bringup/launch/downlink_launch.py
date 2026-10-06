import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Teste local sem rede: joystick ligado diretamente ao safety gate e ao controlo do veículo.


def generate_launch_description():

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description="Joystick device ID (Logitech or Xbox)"
    )
    device_id_config = LaunchConfiguration('device_id')

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='xbox',
        description="Controller type: 'rs50' (default), 'g923' or 'xbox'"
    )
    controller = LaunchConfiguration('controller')

    max_velocity_arg = DeclareLaunchArgument(
        'max_vlc',
        default_value='10.0',
        description="Max velocity in km/h at full throttle"
    )
    max_velocity = LaunchConfiguration('max_vlc')

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

    command_gate_test_node = Node(
        package="rs_interface",
        executable='cpmmand_gate_forsingletest',
        name='command_gate',
        output='screen',
        parameters=[{'max_vlc': ParameterValue(max_velocity, value_type=float)}]
    )

    # métricas falsas para o topic_monitor ficar em OK
    fake_network_health = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-r', '50',
            '/metrics/network/teleop_commands',
            'teleop_msgs/msg/NetworkMetrics',
            '{latency_ms: 1.0, lost_pkg: 0}'
        ],
        output='log'
    )

    topic_monitor_node = Node(
        package='vh_teleop_to_autoware',
        executable='topic_monitor',
        name='topic_monitor',
        output='screen'
    )

    safety_gate_node = Node(
        package='vh_teleop_to_autoware',
        executable='safety_gate',
        name='safety_gate',
        output='screen'
    )

    control_node = Node(
        package='vh_teleop_to_autoware',
        executable='control',
        name='control',
        output='screen'
    )

    return LaunchDescription([
        device_id_arg,
        controller_arg,
        max_velocity_arg,
        joy_node,
        throttle_node,
        rs50_teleop_node,
        g923_teleop_node,
        xbox_teleop_node,
        command_gate_test_node,
        fake_network_health,
        topic_monitor_node,
        safety_gate_node,
        control_node,
    ])