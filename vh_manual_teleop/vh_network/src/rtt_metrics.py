#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading

class RttEchoer(Node):
    def __init__(self):
        super().__init__('rtt_echoer')
        self.declare_parameter('allowed_ip', '10.0.0.2')
        self.declare_parameter('port', 5007)
        self.allowed_ip = self.get_parameter('allowed_ip').value
        self.port       = self.get_parameter('port').value

        self.sock_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_.bind(('0.0.0.0', self.port))
        self.sock_.settimeout(0.5)

        threading.Thread(target=self.echo_loop, daemon=True).start()

        self.get_logger().info(f"RTT Echoer iniciado na porta {self.port}")

    def echo_loop(self):
        while rclpy.ok():
            try:
                data, addr = self.sock_.recvfrom(65535)
                if addr[0] != self.allowed_ip:
                    continue
                self.sock_.sendto(data, addr)  # echo puro
            except socket.timeout:
                continue
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f"Erro: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RttEchoer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock_.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()