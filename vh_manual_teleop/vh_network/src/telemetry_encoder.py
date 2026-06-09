#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
import socket
from msg_manual_teleop.msg import TelemetryState

class TelemetryEncoder(Node):
    def __init__(self):
        super().__init__('telemetry_encoder')
        self.declare_parameter('ip_address', '10.0.0.2')
        self.declare_parameter('port', 5006)

        self.target_ip = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value 

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.create_subscription(
            TelemetryState,
            '/telemetry/state',
            self.telemetry_callback,
            10
        )

        self.get_logger().info(f"Telemetry Encoder → {self.target_ip}:{self.port}")

    def telemetry_callback(self, msg):
        try:
            msg.latest_msg.header.stamp = self.get_clock().now().to_msg()

            data = serialize_message(msg)
            self.sock.sendto(data, (self.target_ip, self.port))
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryEncoder()
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