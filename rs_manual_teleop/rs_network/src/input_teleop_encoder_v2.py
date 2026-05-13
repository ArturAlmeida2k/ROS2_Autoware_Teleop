#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import struct
import time

# Importar a tua mensagem customizada em vez das std_msgs
from msg_manual_teleop.msg import TeleopCommand

class InputTeleopEncoder(Node):
    def __init__(self):
        super().__init__('input_teleop_encoder')
        self.target_ip = "10.0.0.1"
        self.port = 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.data = {
            'vlc': 0.0, 'steer': 0.0, 'brake': 0.0,
            'gear': 0, 'signal': 1, 'engage': False
        }
        
        # Variável para guardar o timestamp do ROS
        self.latest_timestamp = time.time()

        # Substituir as 6 subscrições por apenas 1
        self.create_subscription(
            TeleopCommand, 
            '/teleop/command', 
            self.command_callback, 
            10
        )

        self.create_timer(0.02, self.send_packet)  # 50Hz
        self.get_logger().info(f"Encoder iniciado → {self.target_ip}:{self.port}")

    def command_callback(self, msg):
        # Atualiza o dicionário com os valores da mensagem recebida
        self.data['vlc'] = msg.target_velocity
        self.data['steer'] = msg.target_steering_angle
        self.data['brake'] = msg.brake_factor
        self.data['gear'] = msg.gear
        self.data['signal'] = msg.turn_signal
        self.data['engage'] = msg.engage_command
        
        # Extrair a hora da mensagem (segundos + nanosegundos convertidos para segundos)
        self.latest_timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

    def send_packet(self):
        try:
            # 'd' = double (timestamp), 'fff' = floats, 'ii' = ints, '?' = bool
            packet = struct.pack('dfffii?',
                self.latest_timestamp,  # Usa a hora exata da leitura original do volante
                self.data['vlc'],
                self.data['steer'],
                self.data['brake'],
                self.data['gear'],
                self.data['signal'],
                self.data['engage']
            )
            self.sock.sendto(packet, (self.target_ip, self.port))
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InputTeleopEncoder()
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