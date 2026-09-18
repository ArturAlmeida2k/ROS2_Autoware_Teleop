import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description="ID do dispositivo (Logitech ou Xbox) no sistema"
    )
    device_id_config = LaunchConfiguration('device_id')

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='xbox',
        description="Escolher o tipo de controlador usado para controlar o carro: 'rs50'(default), 'g923', ou 'xbox'"
    )
    controller = LaunchConfiguration('controller')

    # =========================================================================
    # Captura do input físico
    # =========================================================================
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

    # =========================================================================
    # Lado RS: filtragem
    # =========================================================================
    # Versão de teste, sem verificação de telemetria (ver
    # cpmmand_gate_forsingletest.cpp) — sem isto nada passaria, porque não
    # há nenhum /teleop/telemetry real a chegar sem os nós de rede.
    command_gate_test_node = Node(
        package="rs_interface",
        executable='cpmmand_gate_forsingletest',
        name='command_gate',
        output='screen'
    )

    # =========================================================================
    # "Rede saudável" falsa — só para desbloquear o topic_monitor, que de
    # outra forma nunca recebe nada (isso normalmente vem do
    # input_teleop_decoder, que aqui não está a correr) e o safety_gate
    # fica preso em STATE_ERROR para sempre. topic_monitor e safety_gate
    # em si não são tocados, correm exatamente como no sistema real.
    fake_network_health = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '-r', '50',
            '/metrics/network/teleop_commands',
            'teleop_msgs/msg/NetworkMetrics',
            '{latency_ms: 1.0, lost_pkg: 0}'
        ],
        output='screen'
    )

    # =========================================================================
    # Lado VH: segurança e ligação ao Autoware
    # (assume-se que o Autoware/AWSIM já está a correr à parte — não faz
    # parte deste launch, tal como não faz parte de nenhum dos pacotes
    # desenvolvidos neste trabalho.)
    # =========================================================================
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