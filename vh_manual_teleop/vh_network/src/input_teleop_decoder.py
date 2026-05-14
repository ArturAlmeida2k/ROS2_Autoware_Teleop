#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
import socket
import struct
import time

class InputTeleopDecoder(Node):
    def __init__(self):
        super().__init__('input_teleop_decoder')

        self.declare_parameter('ip_address', '10.0.0.2')
        self.allowed_ip = self.get_parameter('ip_address').value

        self.declare_parameter('port', 5005)
        self.port = self.get_parameter('port').value

        self.pub_vlc        = self.create_publisher(Float32, '/teleop/target_velocity', 10)
        self.pub_steer      = self.create_publisher(Float32, '/teleop/target_steering_angle', 10)
        self.pub_brake      = self.create_publisher(Float32, '/teleop/brake_factor', 10)
        self.pub_gear       = self.create_publisher(Int32,   '/teleop/gear_change', 10)
        self.pub_turn       = self.create_publisher(Int32,   '/teleop/turn_signal', 10)
        self.pub_engage     = self.create_publisher(Bool,    '/teleop/engage_command', 10)
        self.pub_delay      = self.create_publisher(Float32, '/teleop/network_delay_ms', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.setblocking(False)

        self.create_timer(0.01, self.receive_packet)  # 100Hz 
        self.get_logger().info(f"Decoder iniciado na porta {self.port}. A aceitar apenas comandos do IP: {self.allowed_ip}")

    def receive_packet(self):
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                sender_ip = addr[0]

                if sender_ip != self.allowed_ip:
                    continue

                send_time, vlc, steer, brake, gear, signal, engage = struct.unpack('dfffii?', data)

                delay_ms = (time.time() - send_time) * 1000

                self.pub_vlc.publish(Float32(data=vlc))
                self.pub_steer.publish(Float32(data=steer))
                self.pub_brake.publish(Float32(data=brake))
                self.pub_gear.publish(Int32(data=gear))
                self.pub_turn.publish(Int32(data=signal))
                self.pub_engage.publish(Bool(data=engage))
                self.pub_delay.publish(Float32(data=float(delay_ms)))

            except BlockingIOError:
                break
            except Exception as e:
                self.get_logger().error(f"Erro: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = InputTeleopDecoder()
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