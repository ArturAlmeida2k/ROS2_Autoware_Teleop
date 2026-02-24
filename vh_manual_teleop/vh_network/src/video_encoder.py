#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class VideoEncoder(Node):
    def __init__(self):
        super().__init__('video_encoder')
        self.bridge = CvBridge()
        self.out = None
        
        self.rs_ip = '10.0.0.2' 
        self.port = 5006
        self.fps = 30
        
        # Define o perfil QoS para coincidir com o do AWSIM
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Aplica o perfil à subscrição
        self.sub = self.create_subscription(
            Image, 
            '/sensing/camera/traffic_light/image_raw', 
            self.encode_callback, 
            qos_profile)
        
        
        self.get_logger().info(f"Nó Encoder iniciado. A enviar para {self.rs_ip}:{self.port}")

    def encode_callback(self, msg):
        start_time = time.time()
        
        # 1. Converter imagem
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = cv_image.shape

        # 2. Inicializar o pipeline GStreamer (apenas no primeiro frame)
        if self.out is None:
            pipeline = (
                f"appsrc ! videoconvert ! video/x-raw,format=I420 ! "
                f"nvh264enc preset=low-latency rc-mode=cbr bitrate=5000 zerolatency=true ! "
                f"rtph264pay config-interval=1 pt=96 ! "
                f"udpsink host={self.rs_ip} port={self.port} sync=false"
            )
            self.out = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, self.fps, (width, height), True)
            if not self.out.isOpened():
                self.get_logger().error("Falha ao abrir o pipeline GStreamer.")
                return

        # 3. Adicionar o Timestamp visual para cálculo de latência end-to-end
        current_time_str = f"Time: {time.time():.3f}"
        cv2.putText(cv_image, current_time_str, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 4. Enviar para a GPU e depois UDP
        self.out.write(cv_image)
        
        # 5. Calcular tempo de processamento local
        process_time_ms = (time.time() - start_time) * 1000
        self.get_logger().info(f"Latência de codificação: {process_time_ms:.1f} ms") 

    def destroy_node(self):
        if self.out is not None:
            self.out.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()