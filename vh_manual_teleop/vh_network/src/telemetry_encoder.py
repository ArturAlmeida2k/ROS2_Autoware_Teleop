#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from msg_manual_teleop.msg import TelemetryState
import socket
import struct
import time

class TelemetryEncoder(Node):
    def __init__(self):
        super().__init__('telemetry_encoder')

        self.target_ip = "10.0.0.2"
        self.port      = 5006
        self.sock      = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.state = TelemetryState()

        self.create_subscription(
            TelemetryState,
            '/telemetry/state',
            self.on_telemetry,
            10
        )

        self.create_timer(0.02, self.send_packet)  # 50Hz
        self.get_logger().info(f"Emissor UDP iniciado → {self.target_ip}:{self.port}")

    def on_telemetry(self, msg: TelemetryState):
        self.state = msg

    def send_packet(self):
        try:
            # Formato: timestamp(d) velocity(f) gear(i) mode(i) turn_signal(i) hazard(i) engaged(?)
            # Total: 8 + 4 + 4 + 4 + 4 + 4 + 1 = 29 bytes
            packet = struct.pack('dfiiii?',
                time.time(),
                self.state.velocity_kmh,
                self.state.gear,
                self.state.mode,
                self.state.turn_signal,
                self.state.hazard,
                self.state.engaged
            )
            self.sock.sendto(packet, (self.target_ip, self.port))
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar pacote UDP: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()