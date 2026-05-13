#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
import socket
import struct
import threading

# Importações das mensagens
from sensor_msgs.msg import PointCloud2
from autoware_perception_msgs.msg import DetectedObjects

class PerceptionDecoder(Node):
    def __init__(self):
        super().__init__('perception_decoder')
        
        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10)
        self.obj_pub = self.create_publisher(DetectedObjects, '/perception/object_recognition/detection/clustering/objects_received', 10)
        
        self.host = '0.0.0.0'
        self.pc_port = 5011
        self.obj_port = 5012
        
        # Iniciar threads para escutar as duas portas simultaneamente
        threading.Thread(target=self.setup_server, args=(self.pc_port, self.process_pc), daemon=True).start()
        threading.Thread(target=self.setup_server, args=(self.obj_port, self.process_obj), daemon=True).start()

    def setup_server(self, port, process_callback):
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind((self.host, port))
        server_sock.listen(1)
        self.get_logger().info(f"Servidor à escuta na porta {port}...")
        
        while rclpy.ok():
            try:
                conn, addr = server_sock.accept()
                self.get_logger().info(f"Conexão aceite na porta {port} de {addr}")
                process_callback(conn)
            except Exception as e:
                self.get_logger().error(f"Erro no servidor (porta {port}): {e}")

    def process_pc(self, conn):
        self.receive_loop(conn, PointCloud2, self.pc_pub)

    def process_obj(self, conn):
        self.receive_loop(conn, DetectedObjects, self.obj_pub)

    def receive_loop(self, conn, msg_type, publisher):
        try:
            while rclpy.ok():
                raw_msglen = self.recvall(conn, 4)
                if not raw_msglen:
                    break
                msglen = struct.unpack('>I', raw_msglen)[0]
                
                data = self.recvall(conn, msglen)
                if not data:
                    break
                    
                msg = deserialize_message(data, msg_type)
                publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erro ao receber dados: {e}")
        finally:
            conn.close()

    def recvall(self, sock, n):
        data = bytearray()
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return bytes(data)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()