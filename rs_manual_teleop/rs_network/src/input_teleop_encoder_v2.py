#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import socket
from msg_manual_teleop.msg import TeleopCommand

class InputTeleopEncoder(Node):
    def __init__(self):
        super().__init__('input_teleop_encoder')
        self.declare_parameter('ip_address', '10.0.0.1')
        self.declare_parameter('port', 5005)

        self.target_ip = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.latest_msg = None

        self.create_subscription(TeleopCommand, '/teleop/command', self.command_callback, 10)
        self.create_timer(0.02, self.send_packet)  # 50Hz

        self.get_logger().info(f"Encoder iniciado → a enviar para {self.target_ip}:{self.port}")

    def command_callback(self, msg):
        self.latest_msg = msg

    def send_packet(self):
        if self.latest_msg is None:
            return
        try:
            self.latest_msg.header.stamp = self.get_clock().now().to_msg()
            
            data = serialize_message(self.latest_msg)
            self.sock.sendto(data, (self.target_ip, self.port))
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