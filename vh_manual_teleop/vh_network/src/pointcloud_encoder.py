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

FRAME_MAGIC = b'PCF1'
HEADER = struct.Struct('>4sI')


class PointCloudEncoder(Node):
    def __init__(self):
        super().__init__('pointcloud_encoder')

        self.declare_parameter('ip_address', '10.0.0.2')
        self.declare_parameter('port', 5011)
        self.declare_parameter(
            'input_topic', '/perception/obstacle_segmentation/pointcloud')
        self.declare_parameter('max_rate_hz', 5.0)      # 0 = sem limite
        self.declare_parameter('connect_timeout', 2.0)
        self.declare_parameter('send_timeout', 1.0)
        self.declare_parameter('reconnect_delay', 1.0)

        p = self.get_parameter
        self.target = (p('ip_address').value, p('port').value)
        self.connect_timeout = p('connect_timeout').value
        self.send_timeout = p('send_timeout').value
        self.reconnect_delay = p('reconnect_delay').value

        rate = p('max_rate_hz').value
        self.min_period = (1.0 / rate) if rate > 0 else 0.0
        self._last_accept = 0.0

        self.q = queue.Queue(maxsize=1)
        self.sock = None
        self._running = True
        self._sent = 0
        self._bytes = 0
        self._dropped = 0
        self._t0 = time.monotonic()

        # Só interessa a nuvem mais recente.
        qos = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(
            PointCloud2, p('input_topic').value, self.on_pointcloud, qos)

        self._thread = threading.Thread(target=self._sender_loop, daemon=True)
        self._thread.start()
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f"PointCloud Encoder → {self.target[0]}:{self.target[1]} "
            f"({p('input_topic').value}, max {rate} Hz)")

    # ------------------------------------------------------------------
    def on_pointcloud(self, msg):
        """Callback rápido: serializa e entrega à thread, nunca bloqueia."""
        now = time.monotonic()
        if self.min_period and (now - self._last_accept) < self.min_period:
            return
        self._last_accept = now

        data = serialize_message(msg)

        try:
            self.q.put_nowait(data)
        except queue.Full:
            try:
                self.q.get_nowait()          # deita fora a frame velha
                self._dropped += 1
            except queue.Empty:
                pass
            try:
                self.q.put_nowait(data)
            except queue.Full:
                self._dropped += 1

    # ------------------------------------------------------------------
    def _connect(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.connect_timeout)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.connect(self.target)
        s.settimeout(self.send_timeout)
        return s

    def _sender_loop(self):
        while self._running:
            if self.sock is None:
                try:
                    self.sock = self._connect()
                    self.get_logger().info("Conectado.")
                except OSError as e:
                    self.get_logger().warn(
                        f"Sem ligação ({e}); nova tentativa em "
                        f"{self.reconnect_delay:.1f}s")
                    self._drain()
                    time.sleep(self.reconnect_delay)
                    continue

            try:
                payload = self.q.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                self.sock.sendall(HEADER.pack(FRAME_MAGIC, len(payload)) + payload)
                self._sent += 1
                self._bytes += len(payload)
            except (OSError, socket.timeout) as e:
                self.get_logger().error(f"Envio falhou ({e}); a reconectar.")
                self._close()

    def _drain(self):
        """Enquanto estamos desligados, o conteúdo da fila já é obsoleto."""
        while True:
            try:
                self.q.get_nowait()
                self._dropped += 1
            except queue.Empty:
                return

    def _close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    # ------------------------------------------------------------------
    def _log_stats(self):
        dt = time.monotonic() - self._t0
        if dt > 0 and (self._sent or self._dropped):
            self.get_logger().info(
                f"tx {self._sent / dt:.1f} Hz | "
                f"{self._bytes / dt * 8 / 1e6:.2f} Mbps | "
                f"{self._bytes / max(1, self._sent) / 1024:.1f} KB/frame | "
                f"descartadas {self._dropped}")
        self._sent = self._bytes = self._dropped = 0
        self._t0 = time.monotonic()

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