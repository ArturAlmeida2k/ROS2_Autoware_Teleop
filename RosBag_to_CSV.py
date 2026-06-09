#!/usr/bin/env python3
import sys
import argparse
from pathlib import Path
import pandas as pd
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

def main():
    # 1. Configurar os argumentos do terminal
    parser = argparse.ArgumentParser(description="Extrai métricas de rede de um ROS 2 Bag para um ficheiro Excel.")
    parser.add_argument("caminho_bag", type=str, help="Caminho (relativo ou absoluto) para a pasta do rosbag")
    args = parser.parse_args()

    # 2. Processar os caminhos com o pathlib
    bag_path = Path(args.caminho_bag).resolve()
    
    if not bag_path.exists():
        print(f"Erro: A pasta '{bag_path}' não foi encontrada.")
        sys.exit(1)

    # Subir dois níveis para sair da pasta da sessão e da pasta 'bags'
    # Ex: .../autoware_workspace/bags/sessao_20260609 -> .../autoware_workspace/
    output_dir = bag_path.parent.parent
    
    # Criar o nome do Excel baseado no nome da pasta do bag
    output_excel = output_dir / f"{bag_path.name}_metricas.xlsx"

    # 3. Registar a mensagem customizada
    typestore = get_typestore(Stores.ROS2_HUMBLE)
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
    
    # Confirma se o tópico é este que pretendes analisar
    target_topic = '/metrics/telemetry'

    print(f"A ler o bag em: {bag_path}")

    # 4. Ler o Bag
    try:
        with Reader(bag_path) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == target_topic:
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                    rows.append({
                        'id':           msg.id,
                        'send_time':    msg.send_time,
                        'receive_time': msg.receive_time,
                        'latency_ms':   msg.latency,
                        'lost_pkg':     msg.lost_pkg,
                    })
    except Exception as e:
        print(f"Erro ao processar o bag: {e}")
        sys.exit(1)

    print(f"Mensagens lidas: {len(rows)}")

    # 5. Guardar em Excel
    if len(rows) == 0:
        print(f"Bag vazio ou o tópico '{target_topic}' não foi encontrado.")
    else:
        df = pd.DataFrame(rows)
        # Exportar para Excel (.xlsx)
        df.to_excel(output_excel, index=False)
        
        print(f"\nSucesso! Ficheiro Excel criado em:\n -> {output_excel}\n")
        print("--- Estatísticas Básicas ---")
        print(df.describe())

if __name__ == '__main__':
    main()