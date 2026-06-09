from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from pathlib import Path
import pandas as pd

typestore = get_typestore(Stores.ROS2_HUMBLE)

# Registar a mensagem customizada
msg_definition = """
uint32 id
float64 send_time
float64 receive_time
float32 latency
int32 lost_pkg
"""

add_types = get_types_from_msg(msg_definition, 'msg_manual_teleop/msg/NetworkMetrics')
typestore.register(add_types)

rows = []

with Reader('/home/artur/bags/sessao_20260609_175559') as reader:
    print("Tópicos no bag:")
    for connection in reader.connections:
        print(f"  {connection.topic}  →  {connection.msgtype}")

    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == '/metrics/telemetry':
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
            rows.append({
                'id':           msg.id,
                'send_time':    msg.send_time,
                'receive_time': msg.receive_time,
                'latency_ms':   msg.latency,
                'lost_pkg':     msg.lost_pkg,
            })

print(f"\nMensagens lidas: {len(rows)}")

if len(rows) == 0:
    print("Bag vazio ou tópico não encontrado.")
else:
    df = pd.DataFrame(rows)
    df.to_excel('metricas.xlsx', index=False)
    print(df.describe())