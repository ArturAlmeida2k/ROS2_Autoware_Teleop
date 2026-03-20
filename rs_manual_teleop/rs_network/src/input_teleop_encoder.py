#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
import socket
import struct

class InputTeleopEncoder(Node):
    def __init__(self):
        super().__init__('input_teleop_encoder')
        self.target_ip = "10.0.0.1"  
        self.port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Estado interno
        self.data = {'vlc': 0.0, 'steer': 0.0, 'brake': 0.0, 'gear': 0, 'signal': 1,'engage': False}

        # Subscreve aos tópicos 
        self.create_subscription(Float32, '/teleop/target_velocity', lambda msg: self.update('vlc', msg.data), 10)
        self.create_subscription(Float32, '/teleop/target_steering_angle', lambda msg: self.update('steer', msg.data), 10)
        self.create_subscription(Float32, '/teleop/brake_factor', lambda msg: self.update('brake', msg.data), 10)
        self.create_subscription(Int32, '/teleop/gear_change', lambda msg: self.update('gear', msg.data), 10)
        self.create_subscription(Int32, '/teleop/turn_signal', lambda msg: self.update('signal', msg.data), 10)
        self.create_subscription(Bool, '/teleop/engage_command', lambda msg: self.update('engage', msg.data), 10)


        # Timer para enviar a 50Hz
        self.create_timer(0.02, self.send_packet)
        self.get_logger().info(f"Nó Encoder iniciado. Enviando para {self.target_ip}:{self.port}")

    def update(self, key, val):
        self.data[key] = val

    def send_packet(self):
        try:
            #floats (f), 2 int (i), 1 bool (?) 
            packet = struct.pack('fffii?', 
                self.data['vlc'], self.data['steer'], self.data['brake'], 
                self.data['gear'], self.data['signal'], self.data['engage'])
            self.sock.sendto(packet, (self.target_ip, self.port))
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar UDP: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InputTeleopEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()