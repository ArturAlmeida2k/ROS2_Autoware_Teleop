#!/usr/bin/env python3

import socket
 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
 
from msg_manual_teleop.msg import NetworkMetrics
 
RECV_BUF = 65535
 
 
class PointCloudDecoder(Node):
    def __init__(self):
        super().__init__('pointcloud_decoder')
 
        self.declare_parameter('ip_address', '10.0.0.1')
        self.declare_parameter('port', 5011)
        self.declare_parameter('output_topic', '/teleop/pointcloud')
 
        self.allowed_ip = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value
        output_topic = self.get_parameter('output_topic').value
 
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_pointcloud = self.create_publisher(PointCloud2, output_topic, qos)
 
        metrics_qos = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE)
        self.pub_metrics = self.create_publisher(
            NetworkMetrics, '/metrics/network/pointcloud', metrics_qos)
 
        # O buffer de receção tem de acomodar vários datagramas de ~8 KB, ou
        # uma rajada perde-se enquanto o timer não corre.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)
 
        self.seq_num = 0
 
        self.create_timer(0.01, self.receive_packet)   # 100 Hz
 
        self.get_logger().info(
            f"PointCloud Decoder → {self.port} from {self.allowed_ip}")
 
    def publish_metrics(self, msg_id, rx_time_msg, tx_time_msg):
        metrics_msg = NetworkMetrics()
        metrics_msg.id = msg_id
        metrics_msg.tx = tx_time_msg
        metrics_msg.rx = rx_time_msg
 
        tx_time = Time.from_msg(tx_time_msg)
        rx_time = Time.from_msg(rx_time_msg)
        latency = rx_time - tx_time
        metrics_msg.latency_ms = latency.nanoseconds / 1000000.0
 
        # Sem identificador de sequência na PointCloud2, a perda não é
        # observável aqui; um datagrama fragmentado ou chega inteiro ou não
        # chega de todo.
        metrics_msg.lost_pkg = 0
 
        self.pub_metrics.publish(metrics_msg)
 
    def receive_packet(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(RECV_BUF)
 
                if addr[0] != self.allowed_ip:
                    continue
 
                start_time = self.get_clock().now().to_msg()
 
                msg = deserialize_message(data, PointCloud2)
                sensor_stamp = msg.header.stamp
 
                self.pub_pointcloud.publish(msg)
 
                self.seq_num += 1
                self.publish_metrics(self.seq_num, start_time, sensor_stamp)
 
            except BlockingIOError:
                break
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f"Erro: {e}")
                break
 
 
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDecoder()
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
 
