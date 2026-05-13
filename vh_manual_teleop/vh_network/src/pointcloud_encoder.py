#!/usr/bin/env python3
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.serialization import serialize_message
import socket
import struct

class PointCloudEncoder(Node):
    def __init__(self):
        super().__init__('pointcloud_encoder')

        self.target_ip = "10.0.0.2"  # IP do Computador Destino
        self.port      = 5011        # Porta TCP
        
        # Criação do Socket TCP (SOCK_STREAM em vez de SOCK_DGRAM)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try:
            self.get_logger().info(f"A tentar conectar TCP a {self.target_ip}:{self.port}...")
            self.sock.connect((self.target_ip, self.port))
            self.get_logger().info("Conectado com sucesso.")
        except Exception as e:
            self.get_logger().error(f"Falha ao conectar (O recetor está ligado?): {e}")

        self.create_subscription(
            PointCloud2,
            '/perception/obstacle_segmentation/pointcloud',
            self.on_pointcloud,
            qos_profile_sensor_data
        )

    def on_pointcloud(self, msg: PointCloud2):
        try:
            # 1. Serializar a mensagem ROS 2 completa em bytes
            serialized_msg = serialize_message(msg)
            
            # 2. Calcular o tamanho do payload para enviar primeiro (Cabeçalho de 4 bytes)
            # '>I' significa: Big-endian, Unsigned Integer
            msg_length = struct.pack('>I', len(serialized_msg))
            
            # 3. Enviar o tamanho seguido dos dados exatos
            # sendall garante que os bytes fragmentados chegam todos
            self.sock.sendall(msg_length + serialized_msg)
            
        except BrokenPipeError:
            self.get_logger().error("Conexão TCP perdida.")
        except Exception as e:
            self.get_logger().error(f"Erro ao enviar PointCloud: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.sock.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()