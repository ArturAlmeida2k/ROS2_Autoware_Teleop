#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import socket
from msg_manual_teleop.msg import TeleopCommand

class InputTeleopDecoder(Node):
    def __init__(self):
        super().__init__('input_teleop_decoder')
        self.declare_parameter('ip_address', '10.0.0.2')
        self.allowed_ip = self.get_parameter('ip_address').value
        self.declare_parameter('port', 5005)
        self.port = self.get_parameter('port').value

        self.pub_command = self.create_publisher(TeleopCommand, '/teleop/command', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        self.create_timer(0.01, self.receive_packet)  # 100Hz
        self.get_logger().info(f"Decoder iniciado na porta {self.port}. A aceitar apenas comandos do IP: {self.allowed_ip}")

    def receive_packet(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)
                if addr[0] != self.allowed_ip:
                    continue
                msg = deserialize_message(data, TeleopCommand)
                self.pub_command.publish(msg)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().error(f"Erro: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InputTeleopDecoder()
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