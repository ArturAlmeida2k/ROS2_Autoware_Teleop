#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from std_msgs.msg import Float32
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

        self.pub_latency = self.create_publisher(Float32, '/metrics/latency_cmd_ms', 10)

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

                recv_time = self.get_clock().now()

                msg = deserialize_message(data, TeleopCommand)

                send_time = Time.from_msg(msg.header.stamp)
                latency_ms = (recv_time - send_time).nanoseconds / 1e6

                if 0.0 < latency_ms < 5000.0:
                    lat_msg = Float32()
                    lat_msg.data = float(latency_ms)
                    self.pub_latency.publish(lat_msg)
                    
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