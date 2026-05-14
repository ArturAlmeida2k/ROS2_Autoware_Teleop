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

        self.declare_parameter('ip_address', '10.0.0.1')
        self.declare_parameter('port', 5006)

        self.allowed_ip = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value

        self.pub = self.create_publisher(TelemetryState, '/telemetry/state', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.sock.bind(("0.0.0.0", self.port))
        self.sock.setblocking(False) 

        self.create_timer(0.01, self.receive_packet)  # 100Hz
        self.get_logger().info(f"Decoder UDP iniciado na porta {self.port}. A aceitar telemetria apenas de {self.allowed_ip}")

    def receive_packet(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                sender_ip = addr[0]

                if sender_ip != self.allowed_ip:
                    continue

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