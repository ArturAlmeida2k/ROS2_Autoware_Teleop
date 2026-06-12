#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message, deserialize_message
import socket
import threading
from msg_manual_teleop.msg import NetworkMetrics

class RttProber(Node):
    def __init__(self):
        super().__init__('rtt_prober')
        self.declare_parameter('target_ip', '10.0.0.1')
        self.declare_parameter('port', 5011)
        self.target_ip = self.get_parameter('target_ip').value
        self.port      = self.get_parameter('port').value

        self.seq_ = 0

        self.pub_rtt = self.create_publisher(NetworkMetrics, '/metrics/rtt', 10)

        # socket de envio
        self.send_sock_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # socket de receção do echo
        self.recv_sock_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock_.bind(('0.0.0.0', self.port))
        self.recv_sock_.settimeout(0.5)

        # thread de envio de probes a 10Hz
        threading.Thread(target=self.probe_loop, daemon=True).start()
        # thread de receção de echos
        threading.Thread(target=self.echo_loop,  daemon=True).start()

        self.get_logger().info(f"RTT Prober iniciado → {self.target_ip}:{self.port}")

    def probe_loop(self):
        import time
        while rclpy.ok():
            probe = NetworkMetrics()
            probe.id        = self.seq_
            probe.send_time = self.get_clock().now().nanoseconds / 1e9
            self.seq_ += 1

            data = serialize_message(probe)
            self.send_sock_.sendto(data, (self.target_ip, self.port))
            time.sleep(0.1)  # 10Hz

    def echo_loop(self):
        while rclpy.ok():
            try:
                data, _ = self.recv_sock_.recvfrom(65535)
                recv_time_s = self.get_clock().now().nanoseconds / 1e9

                probe = deserialize_message(data, NetworkMetrics)

                rtt_ms = (recv_time_s - probe.send_time) * 1e3

                metrics = NetworkMetrics()
                metrics.id           = probe.id
                metrics.send_time    = probe.send_time
                metrics.receive_time = recv_time_s
                metrics.latency      = float(rtt_ms)
                metrics.lost_pkg     = 0

                self.pub_rtt.publish(metrics)

            except socket.timeout:
                continue
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f"Erro: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RttProber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_sock_.close()
        node.recv_sock_.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()