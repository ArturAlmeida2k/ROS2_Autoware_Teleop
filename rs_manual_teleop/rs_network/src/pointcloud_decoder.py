#!/usr/bin/env python3
import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2

from msg_manual_teleop.msg import NetworkMetrics

FRAME_MAGIC = b'PCF1'
HEADER = struct.Struct('>4sI')
MAX_FRAME = 32 * 1024 * 1024   # guarda contra um length corrompido


class PointCloudDecoder(Node):
    def __init__(self):
        super().__init__('pointcloud_decoder')

        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', 5011)
        self.declare_parameter('output_topic', '/teleop/pointcloud')

        p = self.get_parameter
        self.bind = (p('bind_address').value, p('port').value)

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(
            PointCloud2, p('output_topic').value, qos)

        metrics_qos = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)
        self.pub_metrics = self.create_publisher(
            NetworkMetrics, '/metrics/network/pointcloud', metrics_qos)

        self._seq = 0
        self._running = True
        self._thread = threading.Thread(target=self._server_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f"PointCloud Decoder → à escuta em {self.bind[1]}")

    # ------------------------------------------------------------------
    def _server_loop(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(self.bind)
        srv.listen(1)
        srv.settimeout(1.0)

        while self._running:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self.get_logger().info(f"Emissor ligado: {addr}")
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            try:
                self._handle(conn)
            except Exception as e:
                self.get_logger().warn(f"Ligação terminada: {e}")
            finally:
                conn.close()
        srv.close()

    def _recv_exact(self, conn, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("socket fechado pelo emissor")
            buf += chunk
        return bytes(buf)

    def _handle(self, conn):
        while self._running:
            magic, length = HEADER.unpack(self._recv_exact(conn, HEADER.size))
            if magic != FRAME_MAGIC or length > MAX_FRAME:
                raise ValueError(f"stream dessincronizado (magic={magic!r})")

            payload = self._recv_exact(conn, length)
            rx_time = self.get_clock().now()

            msg = deserialize_message(payload, PointCloud2)
            self.pub.publish(msg)
            self.publish_metrics(msg.header.stamp, rx_time)

    # ------------------------------------------------------------------
    def publish_metrics(self, tx_time_msg, rx_time):
        m = NetworkMetrics()
        self._seq += 1
        m.id = self._seq
        m.tx = tx_time_msg
        m.rx = rx_time.to_msg()

        latency = rx_time - Time.from_msg(tx_time_msg)
        m.latency_ms = latency.nanoseconds / 1e6
        m.lost_pkg = 0          # TCP não perde: as frames descartadas ficam no emissor

        self.pub_metrics.publish(m)

    def shutdown(self):
        self._running = False
        self._thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()