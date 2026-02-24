#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import time

class VideoDecoder(Node):
    def __init__(self):
        super().__init__('video_decoder')
        self.port = 5006
        
        # Pipeline otimizado para baixa latência em CPU
        # O max-buffers=1 e drop=true impedem que o vídeo crie uma "fila de espera" e ganhe atraso
        pipeline = (
            f"udpsrc port={self.port} caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! "
            f"rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1"
        )
        
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error("Falha ao escutar a porta UDP. Verifica o GStreamer.")
            return
            
        self.get_logger().info(f"Nó Decoder iniciado na porta {self.port}. A aguardar vídeo...")
        
        # Usamos um timer para ler os frames sem bloquear o ROS 2
        self.timer = self.create_timer(0.01, self.receive_and_display)

    def receive_and_display(self):
        start_time = time.time()
        
        # Lê o frame do buffer
        ret, frame = self.cap.read()
        
        if ret:
            # Calcula latência de descodificação local
            process_time_ms = (time.time() - start_time) * 1000
            
            # Escreve a latência de processamento no ecrã do PC B
            info_str = f"Decode Latency: {process_time_ms:.1f} ms"
            cv2.putText(frame, info_str, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Mostra a imagem
            cv2.imshow("Teleop Camera Stream", frame)
            cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoDecoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()