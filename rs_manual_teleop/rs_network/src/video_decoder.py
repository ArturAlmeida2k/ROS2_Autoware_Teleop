#!/usr/bin/env python3
import cv2

# Pipeline otimizado para latência zero
port = 5006
pipeline = (
    f"udpsrc port={port} caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! "
    f"rtph264depay ! avdec_h264 ! videoconvert ! appsink sync=false drop=true max-buffers=1"
)

print(f"A iniciar Decoder na porta {port}...")
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("ERRO: Falha ao escutar a porta UDP. Verifica o GStreamer.")
    exit()

print("A aguardar vídeo... (Pressiona 'q' na janela para sair)")

# Loop contínuo sem bloqueios de timers
while True:
    ret, frame = cap.read()
    
    if ret:
        cv2.imshow("Teleop Camera Stream", frame)
        
    # Atualiza a janela e verifica se a tecla 'q' foi pressionada
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()