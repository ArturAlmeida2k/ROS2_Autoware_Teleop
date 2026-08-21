#!/usr/bin/env python3
"""
pointcloud_compressor  (pacote: vh_network)

Faz TODO o tratamento pesado da nuvem e publica um blob compacto num tópico
ROS local. Não sabe nada sobre rede.

    /perception/.../pointcloud  (PointCloud2, ~1 MB/frame)
        -> [ crop | NaN | voxel | quantize int16 | LZ4 ]
        -> /teleop/pointcloud/compressed  (CompressedPointCloud, ~50-200 KB)

Separar isto do envio permite: (a) gravar em rosbag exatamente o que foi
transmitido, (b) medir o custo de CPU da compressão isoladamente, (c) trocar
o transporte (TCP/UDP/Zenoh) sem tocar no processamento.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

import pcq_codec as codec
from msg_manual_teleop.msg import CompressedPointCloud


class PointCloudCompressor(Node):
    def __init__(self):
        super().__init__('pointcloud_compressor')

        self.declare_parameter('input_topic',
                               '/perception/obstacle_segmentation/pointcloud')
        self.declare_parameter('output_topic', '/teleop/pointcloud/compressed')
        self.declare_parameter('voxel_size', 0.05)      # m (0 = desligado)
        self.declare_parameter('max_range', 40.0)       # m (0 = sem limite)
        self.declare_parameter('min_range', 0.5)        # m — remove pontos do próprio veículo
        self.declare_parameter('z_min', -2.0)
        self.declare_parameter('z_max', 3.0)
        self.declare_parameter('quant_scale', 0.01)     # m/LSB (1 cm)
        self.declare_parameter('send_intensity', False)  # custa ~1 B/ponto
        self.declare_parameter('max_rate_hz', 5.0)      # 0 = sem limite
        self.declare_parameter('compression', 'auto')   # auto|lz4|zlib|none

        p = self.get_parameter
        self.voxel_size = p('voxel_size').value
        self.max_range = p('max_range').value
        self.min_range = p('min_range').value
        self.z_min = p('z_min').value
        self.z_max = p('z_max').value
        self.quant_scale = p('quant_scale').value
        self.send_intensity = p('send_intensity').value
        self.compression = p('compression').value
        rate = p('max_rate_hz').value
        self.min_period = (1.0 / rate) if rate > 0 else 0.0

        # Só interessa a nuvem mais recente -> depth 1, best effort.
        sub_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
        pub_qos = QoSProfile(depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

        self.pub = self.create_publisher(
            CompressedPointCloud, p('output_topic').value, pub_qos)
        self.create_subscription(
            PointCloud2, p('input_topic').value, self.on_pointcloud, sub_qos)

        self._last_send = 0.0
        self._stat_in = 0
        self._stat_out = 0
        self._stat_bytes = 0
        self._stat_cpu = 0.0
        self._stat_t0 = time.monotonic()
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(
            f"Compressor: voxel={self.voxel_size} m, quant={self.quant_scale} m, "
            f"range=[{self.min_range}, {self.max_range}] m, rate<={rate} Hz")

    # ------------------------------------------------------------------
    def on_pointcloud(self, msg: PointCloud2):
        self._stat_in += 1

        # Rate limit: descarta antes de gastar CPU a desempacotar.
        now = time.monotonic()
        if self.min_period and (now - self._last_send) < self.min_period:
            return
        self._last_send = now

        t0 = time.perf_counter()
        try:
            arr = codec.pointcloud2_to_array(msg)
        except Exception as e:
            self.get_logger().error(f"Não consegui interpretar a PointCloud2: {e}")
            return

        names = arr.dtype.names
        if not all(k in names for k in ('x', 'y', 'z')):
            self.get_logger().error(f"Faltam campos x/y/z. Campos: {names}")
            return

        xyz = np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float32)

        inten = None
        if self.send_intensity:
            for key in ('intensity', 'i', 'reflectivity'):
                if key in names:
                    inten = arr[key].astype(np.float32)
                    break

        # 1. Remover NaN/Inf (obrigatório: quase todos os LiDARs os produzem)
        valid = np.isfinite(xyz).all(axis=1)

        # 2. Crop cilíndrico + altura
        if self.max_range > 0 or self.min_range > 0:
            r2 = xyz[:, 0] ** 2 + xyz[:, 1] ** 2
            if self.max_range > 0:
                valid &= r2 <= self.max_range ** 2
            if self.min_range > 0:
                valid &= r2 >= self.min_range ** 2
        valid &= (xyz[:, 2] >= self.z_min) & (xyz[:, 2] <= self.z_max)

        xyz = xyz[valid]
        if inten is not None:
            inten = inten[valid]

        # 3. Filtro voxel
        if self.voxel_size > 0 and len(xyz):
            idx = codec.voxel_downsample(xyz, self.voxel_size)
            xyz = xyz[idx]
            if inten is not None:
                inten = inten[idx]

        # 4. Quantizar + comprimir
        blob = codec.encode(xyz, inten,
                            scale=self.quant_scale,
                            compression=self.compression)

        out = CompressedPointCloud()
        # Preserva a stamp do sensor -> permite medir latência ponta-a-ponta
        # (sensor -> ecrã do operador) e não apenas o tempo de rede.
        out.header = msg.header
        out.num_points = len(xyz)
        out.format = 'PCQ1'
        out.data = blob.tobytes() if isinstance(blob, np.ndarray) else blob
        self.pub.publish(out)

        self._stat_out += 1
        self._stat_bytes += len(blob)
        self._stat_cpu += time.perf_counter() - t0

    # ------------------------------------------------------------------
    def _log_stats(self):
        dt = time.monotonic() - self._stat_t0
        if dt <= 0 or self._stat_out == 0:
            self._stat_t0 = time.monotonic()
            self._stat_in = 0
            return
        self.get_logger().info(
            f"in {self._stat_in / dt:.1f} Hz | out {self._stat_out / dt:.1f} Hz | "
            f"{self._stat_bytes / dt * 8 / 1e6:.2f} Mbps | "
            f"{self._stat_bytes / self._stat_out / 1024:.0f} KB/frame | "
            f"cpu {self._stat_cpu / self._stat_out * 1000:.1f} ms/frame")
        self._stat_in = self._stat_out = self._stat_bytes = 0
        self._stat_cpu = 0.0
        self._stat_t0 = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCompressor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
