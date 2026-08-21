#!/usr/bin/env python3
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2

from msg_manual_teleop.msg import NodeMetrics

UDP_MAX = 65507   # limite de payload de um datagrama UDP sobre IPv4


class PointCloudEncoder(Node):
    def __init__(self):
        super().__init__('pointcloud_encoder')

        self.declare_parameter('ip_address', '10.0.0.2')
        self.declare_parameter('port', 5011)
        self.declare_parameter(
            'input_topic', '/perception/obstacle_segmentation/pointcloud')

        self.target_ip = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value
        input_topic = self.get_parameter('input_topic').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Só interessa a nuvem mais recente.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            PointCloud2, input_topic, self.pointcloud_callback, qos)

        metrics_qos = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.pub_metrics = self.create_publisher(
            NodeMetrics, '/metrics/pointcloud_encoder', metrics_qos)

        self.seq_num = 1

        self.get_logger().info(
            f"PointCloud Encoder → {self.target_ip}:{self.port} ({input_topic})")

    def publish_metrics(self, msg_id, rx_time_msg, tx_time_msg):
        metrics_msg = NodeMetrics()
        metrics_msg.id = msg_id
        metrics_msg.tx = tx_time_msg
        metrics_msg.rx = rx_time_msg

        rx_time = Time.from_msg(rx_time_msg)
        tx_time = Time.from_msg(tx_time_msg)
        latency = rx_time - tx_time
        metrics_msg.latency_ms = latency.nanoseconds / 1000000.0

        self.pub_metrics.publish(metrics_msg)

    def pointcloud_callback(self, msg):
        try:
            start_time = self.get_clock().now().to_msg()
            sensor_stamp = msg.header.stamp

            data = serialize_message(msg)

            if len(data) > UDP_MAX:
                self.get_logger().warn(
                    f"Nuvem de {len(data)} B excede o limite de um datagrama "
                    f"UDP ({UDP_MAX} B); descartada.",
                    throttle_duration_sec=5.0)
                return

            self.sock.sendto(data, (self.target_ip, self.port))

            self.publish_metrics(self.seq_num, start_time, sensor_stamp)
            self.seq_num += 1

        except Exception as e:
            self.get_logger().error(f"Erro ao enviar: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudEncoder()
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