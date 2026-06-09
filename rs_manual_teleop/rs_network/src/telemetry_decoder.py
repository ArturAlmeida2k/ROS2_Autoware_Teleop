#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.time import Time
import socket
from msg_manual_teleop.msg import TelemetryState
from msg_manual_teleop.msg import NetworkMetrics

class TelemetryDecoder(Node):
    def __init__(self):
        super().__init__('telemetry_decoder')
        self.declare_parameter('ip_address', '10.0.0.1')
        self.declare_parameter('port', 5006)
        self.allowed_ip = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value

        self.expected_id_ = None

        self.pub_telemetry = self.create_publisher(TelemetryState, '/telemetry/state', 10)
        self.pub_metrics = self.create_publisher(NetworkMetrics, '/metrics/telemetry', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        self.create_timer(0.005, self.receive_packet)  # 200Hz

        self.get_logger().info(f"Telemetry Decoder → {self.port} from {self.allowed_ip}")

    def receive_packet(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(65535)

                if addr[0] != self.allowed_ip:
                    continue

                recv_time = self.get_clock().now()
                msg = deserialize_message(data, TelemetryState)

                self.pub_telemetry.publish(msg)

                send_time = Time.from_msg(msg.header.stamp)
                send_time_s  = send_time.nanoseconds / 1e9
                recv_time_s  = recv_time.nanoseconds / 1e9
                latency_ms   = (recv_time_s - send_time_s) * 1e3

                lost = 0
                if self.expected_id_ is not None and msg.id != self.expected_id_:
                    lost = int(msg.id - self.expected_id_)

                self.expected_id_ = msg.id + 1

                metrics_msg = NetworkMetrics()
                metrics_msg.id           = msg.id
                metrics_msg.send_time    = float(send_time_s)
                metrics_msg.receive_time = float(recv_time_s)
                metrics_msg.latency      = float(latency_ms)
                metrics_msg.lost_pkg     = lost

                self.pub_metrics.publish(metrics_msg)

            except BlockingIOError:
                break
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f"Erro: {e}")
                break

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryDecoder()
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