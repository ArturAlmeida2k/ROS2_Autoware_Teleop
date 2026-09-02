#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.time import Time
import socket

from std_msgs.msg import UInt8

from msg_manual_teleop.msg import TeleopCommand
from msg_manual_teleop.msg import NetworkMetrics

class InputTeleopDecoder(Node):
    def __init__(self):
        super().__init__('input_teleop_decoder')
        self.declare_parameter('ip_address', '10.0.0.2')
        self.declare_parameter('port', 5005)
        
        self.allowed_ip = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value

        self.expected_id_ = None

        # 1. Publisher original de comandos
        self.pub_command = self.create_publisher(TeleopCommand, '/teleop/command', 10)
        
        # 2. Publisher dedicado ao estado do Uplink
        self.pub_uplink = self.create_publisher(UInt8, '/teleop/uplink_mode', 10)

        self.pub_metrics = self.create_publisher(NetworkMetrics, '/metrics/network/teleop_commands', 10)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        self.create_timer(0.005, self.receive_packet)  # 200Hz

        self.get_logger().info(f"Command Decoder → {self.port} from {self.allowed_ip}")

    def publish_metrics(self, msg_id, rx_time_msg, tx_time_msg):
        metrics_msg = NetworkMetrics()
        metrics_msg.id = msg_id

        metrics_msg.tx = tx_time_msg
        metrics_msg.rx = rx_time_msg

        tx_time = Time.from_msg(tx_time_msg)
        rx_time = Time.from_msg(rx_time_msg)
        latency= rx_time - tx_time
        metrics_msg.latency_ms = latency.nanoseconds / 1000000.0

        lost = 0
        if self.expected_id_ is not None and msg_id != self.expected_id_:
            lost = int(msg_id - self.expected_id_)

        self.expected_id_ = msg_id + 1
        metrics_msg.lost_pkg = lost
        
        self.pub_metrics.publish(metrics_msg)

    def receive_packet(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)

                if addr[0] != self.allowed_ip:
                    continue
                
                start_time = self.get_clock().now().to_msg()

                msg = deserialize_message(data, TeleopCommand)

                incoming_stamp = msg.header.stamp
                msg.header.stamp = start_time

                self.pub_command.publish(msg)

                uplink_msg = UInt8()
                uplink_msg.data = msg.uplink_mode
                self.pub_uplink.publish(uplink_msg)

                self.publish_metrics(msg.id, start_time, incoming_stamp)
                
            except BlockingIOError:
                break
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f"Erro: {e}")
                break

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