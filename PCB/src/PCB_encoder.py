#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
import socket
import struct

class TeleopGatewayTX(Node):
    def __init__(self):
        super().__init__('teleop_gateway_tx')
        self.target_ip = "10.0.0.1"  # IP do Túnel do PC A
        self.port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Estado interno
        self.data = {'vlc': 0.0, 'steer': 0.0, 'brake': 0.0, 'gear': 0, 'engage': False}

        # Subscreve os tópicos do teu nó C++
        self.create_subscription(Float32, '/teleop/target_velocity', lambda msg: self.update('vlc', msg.data), 10)
        self.create_subscription(Float32, '/teleop/target_steering_angle', lambda msg: self.update('steer', msg.data), 10)
        self.create_subscription(Float32, '/teleop/brake_factor', lambda msg: self.update('brake', msg.data), 10)
        self.create_subscription(Int32, '/teleop/gear_change', lambda msg: self.update('gear', msg.data), 10)
        self.create_subscription(Bool, '/teleop/engage_command', lambda msg: self.update('engage', msg.data), 10)

        # Timer para enviar a 50Hz
        self.create_timer(0.02, self.send_packet)
        self.get_logger().info(f"Nó Encoder iniciado. Enviando para {self.target_ip}:{self.port}")

    def update(self, key, val):
        self.data[key] = val

    def send_packet(self):
        try:
            # Empacota: 3 floats (f), 1 int (i), 1 bool (?) = 17 bytes
            packet = struct.pack('fffi?', 
                self.data['vlc'], self.data['steer'], self.data['brake'], 
                self.data['gear'], self.data['engage'])
            self.sock.sendto(packet, (self.target_ip, self.port))
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar UDP: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopGatewayTX()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

# ISTO É O QUE FALTA:
if __name__ == '__main__':
    main()