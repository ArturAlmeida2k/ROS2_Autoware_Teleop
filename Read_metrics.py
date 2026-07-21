import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import statistics

def analyze_bag_latencies(bag_path):
    # Inicializar o leitor do rosbag
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    # Abrir o bag
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Erro ao abrir o rosbag: {e}")
        return

    # Obter os tipos de mensagem para cada tópico
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_metadata.name: topic_metadata.type for topic_metadata in topic_types}

    # Dicionário para armazenar as latências de cada tópico
    topic_latencies = {}

    print(f"A ler o bag em '{bag_path}' e a processar mensagens...\n")

    # Iterar sobre todas as mensagens
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        # Obter a classe da mensagem e desserializar
        try:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
        except Exception:
            # Ignora se não conseguir carregar o tipo de mensagem
            continue

        # Verificar se a mensagem tem o campo 'latency_ms'
        if hasattr(msg, 'latency_ms'):
            if topic not in topic_latencies:
                topic_latencies[topic] = []
            topic_latencies[topic].append(msg.latency_ms)

    # Calcular e imprimir as estatísticas (Média, Máx, Mín)
    print("-" * 45)
    print(f"{'Tópico':<30} | {'Mín (ms)':<8} | {'Média (ms)':<10} | {'Máx (ms)':<8}")
    print("-" * 45)
    
    if not topic_latencies:
        print("Nenhuma mensagem com o campo 'latency_ms' encontrada.")
        return

    for topic, latencies in topic_latencies.items():
        if latencies:
            min_lat = min(latencies)
            max_lat = max(latencies)
            mean_lat = statistics.mean(latencies)
            
            print(f"{topic:<30} | {min_lat:<8.2f} | {mean_lat:<10.2f} | {max_lat:<8.2f}")

if __name__ == '__main__':
    # Configurar o parser de argumentos
    parser = argparse.ArgumentParser(description="Extrai estatísticas de latência de um rosbag2.")
    parser.add_argument(
        "bag_path", 
        type=str, 
        help="O caminho para o diretório do rosbag (onde está o metadata.yaml)"
    )
    
    # Processar os argumentos do terminal
    args = parser.parse_args()
    
    # Chamar a função principal
    analyze_bag_latencies(args.bag_path)