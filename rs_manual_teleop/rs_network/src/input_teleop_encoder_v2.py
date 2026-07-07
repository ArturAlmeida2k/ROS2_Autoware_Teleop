#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
import socket
from msg_manual_teleop.msg import TeleopCommand
from msg_manual_teleop.msg import NodeMetrics


class InputTeleopEncoder(Node):
    def __init__(self):
        super().__init__('input_teleop_encoder')
        self.declare_parameter('ip_address', '10.0.0.1')
        self.declare_parameter('port', 5005)

        self.target_ip = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.create_subscription(TeleopCommand, '/teleop/command', self.command_callback, 10)

        metrics_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub_metrics = self.create_publisher(NodeMetrics, '/metrics/command_gate', metrics_qos)


        self.get_logger().info(f"Command Enconder → {self.target_ip}:{self.port}")

    def publish_metrics(self, msg_id, rx_time_msg, tx_time_msg):
        metrics_msg = NodeMetrics()
        metrics_msg.id = msg_id
        
        # 1. Atribuição direta dos tempos absolutos para o rosbag
        metrics_msg.tx = tx_time_msg
        metrics_msg.rx = rx_time_msg

        # 2. Converter de builtin_interfaces para rclpy.time.Time para a matemática
        rx_time = Time.from_msg(rx_time_msg)
        tx_time = Time.from_msg(tx_time_msg)

        # 3. Calcular a diferença e converter para ms (em Python a duração dá-nos nanosegundos diretamente)
        latency = rx_time - tx_time
        metrics_msg.latency_ms = latency.nanoseconds / 1000000.0

        self.pub_metrics.publish(metrics_msg)

    def command_callback(self, msg):
        try:
            start_time = self.get_clock().now().to_msg()
            incoming_stamp = msg.header.stamp

            msg.header.stamp = start_time

            data = serialize_message(msg)
            self.sock.sendto(data, (self.target_ip, self.port))

            self.publish_metrics(msg.id, start_time, incoming_stamp)
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar: {e}")
        

def main(args=None):
    rclpy.init(args=args)
    node = InputTeleopEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()