#!/usr/bin/env python3
import queue
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8

MAX_RATE_HZ = 5.0

FRAME_MAGIC = b'PCF1'
HEADER = struct.Struct('>4sIQI')     # magic, length, ingress_ts_ns, seq_id


class PointCloudEncoder(Node):
    def __init__(self):
        super().__init__('pointcloud_encoder')

        self.declare_parameter('ip_address', '10.0.0.2')
        self.declare_parameter('port', 5011)
        self.target = (self.get_parameter('ip_address').value,
                       self.get_parameter('port').value)

        self.min_period_ns= int(1e9 / MAX_RATE_HZ) if MAX_RATE_HZ > 0 else 0
        self._last_accept_ns = 0.0

        self._active = False
        self._seq = 0

        self.q = queue.Queue(maxsize=1)
        self.sock = None
        self._running = True

        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)

        self.create_subscription(PointCloud2, '/perception/obstacle_segmentation/pointcloud', self.on_pointcloud, qos)

        self.create_subscription(Int8, '/teleop/uplink_mode', self.on_mode, 10)

        self._thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._thread.start()

    # ------------------------------------------------------------------
    def on_mode(self, msg):
        active = (msg.data == 3)
        if active != self._active:
            self._active = active
            if not active:
                self._drain()

    def on_pointcloud(self, msg):
        if not self._active:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self.min_period_ns and (now_ns - self._last_accept_ns) < self.min_period_ns:
                return
        self._last_accept_ns = now_ns

        self._seq = (self._seq + 1) & 0xFFFFFFFF
        seq_id = self._seq
        data = serialize_message(msg)

        try:
            self.q.put_nowait((seq_id, now_ns, data))
        except queue.Full:
            try:
                self.q.get_nowait()          
            except queue.Empty:
                pass
            try:
                self.q.put_nowait((seq_id, now_ns, data))
            except queue.Full:
                pass

    # ------------------------------------------------------------------
    def _connect(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.connect(self.target)
        s.settimeout(1.0)
        return s

    def _sender_loop(self):
        while self._running:
            if self.sock is None:
                try:
                    self.sock = self._connect()
                except OSError:
                    self._drain()
                    time.sleep(1.0)
                    continue

            try:
                seq_id, ingress_ns, payload = self.q.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                self.sock.sendall(
                    HEADER.pack(FRAME_MAGIC, len(payload), ingress_ns, seq_id)
                    + payload)
            except (OSError, socket.timeout):
                self._close()

    def _drain(self):
        while True:
            try:
                self.q.get_nowait()
            except queue.Empty:
                return

    def _close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def shutdown(self):
        self._running = False
        self._thread.join(timeout=2.0)
        self._close()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudEncoder()
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