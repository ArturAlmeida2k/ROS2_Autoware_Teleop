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

from msg_manual_teleop.msg import NetworkMetrics, NodeMetrics

FRAME_MAGIC = b'PCF1'
HEADER = struct.Struct('>4sIQI')       # magic, length, ingress_ts_ns, seq_id
MAX_FRAME = 32 * 1024 * 1024           # guarda contra um length corrompido


class PointCloudDecoder(Node):
    def __init__(self):
        super().__init__('pointcloud_decoder')

        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', 5011)

        p = self.get_parameter
        self.bind = (p('bind_address').value, p('port').value)

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.pub_pointcloud = self.create_publisher(PointCloud2, "/teleop/pointcloud", qos)

        metrics_qos = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT,
                                 history=HistoryPolicy.KEEP_LAST)

        self.pub_network = self.create_publisher(NetworkMetrics, '/metrics/network/pointcloud', metrics_qos)

        self.pub_e2e = self.create_publisher(NodeMetrics, '/metrics/pointcloud_sensor_to_station', metrics_qos)

        self._expected_id = None
        self._running = True
        self._thread = threading.Thread(target=self._server_loop, daemon=True)
        self._thread.start()

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
            self._expected_id = None       
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
            magic, length, ingress_ns, seq_id = HEADER.unpack(
                self._recv_exact(conn, HEADER.size))
            if magic != FRAME_MAGIC or length > MAX_FRAME:
                raise ValueError(f"stream dessincronizado (magic={magic!r})")

            payload = self._recv_exact(conn, length)
            rx_time = self.get_clock().now()

            msg = deserialize_message(payload, PointCloud2)
            sensor_stamp = msg.header.stamp

            msg.header.frame_id = f"{msg.header.frame_id}#{seq_id}"

            self.pub_pointcloud.publish(msg)
            self._publish_metrics(seq_id, sensor_stamp, ingress_ns, rx_time)

    # ------------------------------------------------------------------
    def _publish_metrics(self, seq_id, sensor_stamp, ingress_ns, rx_time):
        rx_msg = rx_time.to_msg()

        # Perda: quantas frames foram descartadas no emissor antes desta.
        lost = 0
        if self._expected_id is not None:
            lost = (seq_id - self._expected_id) & 0xFFFFFFFF
        self._expected_id = (seq_id + 1) & 0xFFFFFFFF

        ingress_time = Time(nanoseconds=ingress_ns, clock_type=rx_time.clock_type)
        net = NetworkMetrics()
        net.id = seq_id
        net.tx = ingress_time.to_msg()
        net.rx = rx_msg
        net.latency_ms = (rx_time - ingress_time).nanoseconds / 1e6
        net.lost_pkg = lost
        self.pub_network.publish(net)

        # Sensor -> estação: inclui o tempo de percepção e de serialização.
        e2e = NodeMetrics()
        e2e.id = seq_id
        e2e.tx = sensor_stamp
        e2e.rx = rx_msg
        e2e.latency_ms = (
            rx_time - Time.from_msg(sensor_stamp)).nanoseconds / 1e6
        self.pub_e2e.publish(e2e)

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