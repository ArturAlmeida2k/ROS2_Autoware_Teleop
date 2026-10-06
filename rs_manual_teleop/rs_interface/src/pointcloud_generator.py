#!/usr/bin/env python3
# Publica uma nuvem de pontos aleatória em /lidar/points, para testar sem o simulador.
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct

class FakePointCloudNode(Node):
    def __init__(self):
        super().__init__('fake_pointcloud_node')
        self.publisher_ = self.create_publisher(PointCloud2, '/lidar/points', 10)
        self.timer = self.create_timer(0.1, self.publish_points) # 10 Hz

        # 10k pontos aleatórios num cubo de -5 a 5 m
        self.points = np.random.uniform(-5.0, 5.0, (10000, 3)).astype(np.float32)

    def publish_points(self):
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'

        msg.height = 1
        msg.width = len(self.points)
        
        # x, y, z em float32
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12 # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        msg.data = self.points.tobytes()
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakePointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()