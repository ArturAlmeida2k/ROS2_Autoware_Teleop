#!/usr/bin/env python3
# Recebe a telemetria do veículo por UDP e publica em /teleop/telemetry.
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import socket
import threading
from teleop_msgs.msg import TelemetryState
from teleop_msgs.msg import NetworkMetrics

class TelemetryDecoder(Node):
    def __init__(self):
        super().__init__('telemetry_decoder')
        self.declare_parameter('ip_address', '10.0.0.1')
        self.declare_parameter('port', 5006)
        
        self.allowed_ip = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value

        self.expected_id_ = None

        # se o id recuar mais do que isto, assume que o emissor reiniciou
        self.RESYNC_THRESHOLD = 1000

        self.pub_telemetry = self.create_publisher(TelemetryState, '/teleop/telemetry', 10)

        metrics_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub_metrics = self.create_publisher(NetworkMetrics, '/metrics/network/telemetry', metrics_qos)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        # thread com socket bloqueante para carimbar o pacote mal chega;
        # o timeout é só para conseguir sair no shutdown
        self.sock.settimeout(0.5)

        self.rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.rx_thread.start()

        self.get_logger().info(f"Telemetry decoder on port {self.port}, from {self.allowed_ip}")

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
        if self.expected_id_ is not None:
            lost = int(msg_id - self.expected_id_)
        self.expected_id_ = msg_id + 1
        metrics_msg.lost_pkg = lost
        
        self.pub_metrics.publish(metrics_msg)

    def receive_loop(self):
        while rclpy.ok():
            try:
                data, addr = self.sock.recvfrom(65535)

                if addr[0] != self.allowed_ip:
                    continue
                
                start_time = self.get_clock().now().to_msg()

                msg = deserialize_message(data, TelemetryState)

                if self.expected_id_ is not None and msg.id < self.expected_id_ - self.RESYNC_THRESHOLD:
                    self.expected_id_ = None

                # pacote atrasado ou repetido
                if self.expected_id_ is not None and msg.id < self.expected_id_:
                    continue

                incoming_stamp = msg.header.stamp

                msg.header.stamp = start_time

                self.pub_telemetry.publish(msg)

                self.publish_metrics(msg.id, start_time, incoming_stamp)

            except socket.timeout:
                continue
            except OSError:
                # socket fechado no shutdown
                break
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f"Receive error: {e}")

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