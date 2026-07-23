import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from datetime import datetime

def generate_launch_description():
   
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='0',
        description="ID do dispositivo (Logitech ou Xbox) no sistema"
    )
    device_id_config = LaunchConfiguration('device_id')

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='rs50',
        description="Escolher o tipo de controlador usado para controlar o carro: 'rs50'(default), 'g923', ou 'xbox'"
    )
    controller = LaunchConfiguration('controller')

    video_arg = DeclareLaunchArgument(
        'video',
        default_value='none',
        description="Selecione o encoder de vídeo: 'none' (nenhum), '1x'(AWSIM), ou '4x'(CARLA)"
    )
    video = LaunchConfiguration('video')

    
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


    
    # 1. Nó do Sistema: Leitura do Joystick com throttle para 50hz
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

    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='joy_throttle',
        arguments=['messages', '/joy', '60.0', '/joy_throttled']
    )

    # 2a. Nó de Mapeamento: Logitech RS50 
    rs50_teleop_node = Node(
        package="rs_interface",
        executable='rs50_teleop_node',
        name='rs50_teleop_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'rs50'"]))
    )

    # 2b. Nó de Mapeamento: Logitech G923 
    g923_teleop_node = Node(
        package="rs_interface",
        executable='g923_teleop_node',
        name='g923_teleop_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'g923'"]))
    )

    # 2c. Nó de Mapeamento: Xbox 
    xbox_teleop_node = Node(
        package="rs_interface",
        executable='xbox_teleop_node',
        name='xbox_teleop_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", controller, "' == 'xbox'"]))
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

    # Nó opcional: video_encoder
    video_decoder_node = Node(
        package='rs_network',
        executable='video_decoder_cpp_v2',
        name='video_decoder_v2',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video, "' == '1x'"])),
        parameters=[{'ip_address': ip_address, 'port': camera_port}]
    )

    # Nó opcional: video_encoder_4x
    video_decoder_4x_node = Node(
        package='rs_network',
        executable='video_decoder_4x_cpp',
        name='video_decoder_4x_cpp',
        output='screen',
        condition=IfCondition(PythonExpression(["'", video, "' == '4x'"])),
        parameters=[{'ip_address': ip_address, 'port': camera_port}]
    )

    # Rosbag
    bag_dir = os.path.expanduser('~/bags')
    os.makedirs(bag_dir, exist_ok=True)  
    
    # Criar um caminho único para este bag consolidado
    bag_metrics_path = os.path.join(bag_dir, f'metrics/{datetime.now().strftime("%Y%m%d_%H%M%S")}')

    rosbag_metrics_node = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '/metrics/telemetry',
            '/metrics/controller',
            '/metrics/command_gate',
            '/metrics/network/telemetry',
            '/metrics/telemetry_decoder',
            '/metrics/telemetry_gui',
            '/metrics/e2e_telemetry_latency',
            '/metrics/full_latency',
            '/metrics/front_camera',
            '--output', bag_metrics_path
        ],
        output='screen'
    )

    return LaunchDescription([
        device_id_arg,
        controller_arg,
        video_arg,
        ip_address_arg,
        input_port_arg,
        telemetry_port_arg,
        camera_port_arg,
        joy_node,
        throttle_node,
        rs50_teleop_node,
        g923_teleop_node,
        xbox_teleop_node,
        command_gate_node,
        input_teleop_encoder_node,
        telemetry_decoder_node,
        video_decoder_node,
        video_decoder_4x_node,
        rosbag_metrics_node
    ])