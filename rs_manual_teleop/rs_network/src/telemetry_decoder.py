#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from msg_manual_teleop.msg import TelemetryState
import socket
import struct

PACKET_FMT  = 'dfiiii?'
PACKET_SIZE = struct.calcsize(PACKET_FMT)

class TelemetryDecoder(Node):
    def __init__(self):
        super().__init__('telemetry_decoder')
        self.pub = self.create_publisher(TelemetryState, '/telemetry/state', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 5006))
        self.sock.setblocking(False)  
        self.create_timer(0.01, self.receive_packet)  # 100Hz
        self.get_logger().info("Decoder UDP iniciado, a escutar na porta 5006...")

    def receive_packet(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                if len(data) != PACKET_SIZE:
                    continue
                timestamp, velocity, gear, mode, turn_signal, hazard, engaged = \
                    struct.unpack(PACKET_FMT, data)
                msg = TelemetryState()
                msg.velocity_kmh = velocity
                msg.gear         = gear
                msg.mode         = mode
                msg.turn_signal  = turn_signal
                msg.hazard       = hazard
                msg.engaged      = engaged
                self.pub.publish(msg)
            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().error(f"Erro ao receber pacote: {e}")

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