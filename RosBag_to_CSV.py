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
    # Novo argumento opcional 'topico'
    parser.add_argument("topico", type=str, nargs='?', default=None, help="Tópico a extrair, ou 'info' para ver os tópicos existentes")
    args = parser.parse_args()

    # 2. Processar os caminhos com o pathlib
    bag_path = Path(args.caminho_bag).resolve()
    
    if not bag_path.exists():
        print(f"Erro: A pasta '{bag_path}' não foi encontrada.")
        sys.exit(1)

    output_dir = bag_path.parent.parent
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

    # 4. Ler o Bag
    try:
        with Reader(bag_path) as reader:
            
            # --- MODO INFO ---
            if args.topico == "info":
                print(f"\n--- Tópicos disponíveis no bag '{bag_path.name}' ---")
                if not reader.connections:
                    print("  (Nenhum tópico encontrado / Bag vazio)")
                for connection in reader.connections:
                    print(f"  Tópico: {connection.topic}  →  Tipo: {connection.msgtype}")
                print("--------------------------------------------------\n")
                sys.exit(0)

            # --- MODO EXTRAÇÃO (Determinar Tópico) ---
            target_topic = args.topico

            if target_topic is None:
                if not reader.connections:
                    print("Erro: O bag está vazio.")
                    sys.exit(1)
                
                # Tentar auto-detetar o tópico com base no tipo da mensagem
                metric_topics = [conn.topic for conn in reader.connections if conn.msgtype == 'msg_manual_teleop/msg/NetworkMetrics']
                
                if metric_topics:
                    target_topic = metric_topics[0]
                    print(f"[Auto-Deteção] Tópico assumido: {target_topic}")
                else:
                    # Se não encontrar a mensagem customizada, usa o primeiro tópico existente
                    target_topic = reader.connections[0].topic
                    print(f"[Aviso] Mensagem NetworkMetrics não detetada. A extrair o 1º tópico existente: {target_topic}")
            else:
                print(f"A extrair o tópico solicitado: {target_topic}")

            print(f"A ler o bag em: {bag_path} ...\n")

            # --- EXTRAÇÃO DE DADOS ---
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

    # 5. Guardar em Excel
    if len(rows) == 0:
        print(f"Bag vazio ou o tópico '{target_topic}' não possui mensagens com este formato.")
    else:
        df = pd.DataFrame(rows)
        
        # Limite de linhas seguro para o Excel
        MAX_ROWS_PER_SHEET = 1000000 
        
        if len(df) > MAX_ROWS_PER_SHEET:
            print(f"\nAviso: O ficheiro tem {len(df)} linhas, o que excede o limite do Excel.")
            print("A dividir os dados em múltiplas folhas (Sheets) no mesmo ficheiro...")
            
            # Escrever em múltiplas folhas
            with pd.ExcelWriter(output_excel, engine='openpyxl') as writer:
                for i in range(0, len(df), MAX_ROWS_PER_SHEET):
                    chunk = df.iloc[i:i + MAX_ROWS_PER_SHEET]
                    sheet_name = f'Metricas_Parte_{i // MAX_ROWS_PER_SHEET + 1}'
                    chunk.to_excel(writer, sheet_name=sheet_name, index=False)
                    print(f" -> Guardada {sheet_name} com {len(chunk)} linhas.")
        else:
            # Escrever normalmente se for abaixo do limite
            df.to_excel(output_excel, index=False)
        
        print(f"\nSucesso! Ficheiro Excel criado em:\n -> {output_excel}\n")
        print("--- Estatísticas Básicas ---")
        print(df.describe())

if __name__ == '__main__':
    main()