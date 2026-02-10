#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Int32
import socket
import struct

class TeleopGatewayRX(Node):
    def __init__(self):
        super().__init__('teleop_gateway_rx')
        self.pub_vlc = self.create_publisher(Float32, '/teleop/target_velocity', 10)
        self.pub_steer = self.create_publisher(Float32, '/teleop/target_steering_angle', 10)
        self.pub_brake = self.create_publisher(Float32, '/teleop/brake_factor', 10)
        self.pub_gear = self.create_publisher(Int32, '/teleop/gear_change', 10)
        self.pub_engage = self.create_publisher(Bool, '/teleop/engage_command', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 5005))
        self.sock.setblocking(False)
        
        self.create_timer(0.01, self.receive_packet)

    def receive_packet(self):
        try:
            data, _ = self.sock.recvfrom(1024)
            vlc, steer, brake, gear, engage = struct.unpack('fffi?', data)
            
            # Republica internamente no PC A
            self.pub_vlc.publish(Float32(data=vlc))
            self.pub_steer.publish(Float32(data=steer))
            self.pub_brake.publish(Float32(data=brake))
            self.pub_gear.publish(Int32(data=gear))
            self.pub_engage.publish(Bool(data=engage))
        except BlockingIOError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TeleopGatewayRX()
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()