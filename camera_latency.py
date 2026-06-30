import pandas as pd
import numpy as np

def corrigir_wrap_around(df):
    """
    Deteta quebras drásticas negativas no número de sequência e adiciona 
    65536 cumulativamente para linearizar o índice.
    """
    seqs = df['seq'].values.astype(int)
    corricoes = np.zeros_like(seqs)
    ciclo_atual = 0
    
    for i in range(1, len(seqs)):
        # Se a diferença for fortemente negativa, ocorreu um wrap-around
        if seqs[i] - seqs[i-1] < -30000:
            ciclo_atual += 1
        corricoes[i] = ciclo_atual
        
    df['seq_absoluto'] = seqs + (corricoes * 65536)
    return df

# 1. Carregar ficheiros
df_tx = pd.read_csv('tx_5007.csv', sep=',', names=['seq', 'time_tx'], header=None)
df_rx = pd.read_csv('rx_5007.csv', sep=',', names=['seq', 'time_rx'], header=None)

df_tx = df_tx.dropna()
df_rx = df_rx.dropna()

# 2. Aplicar a correção de wrap-around de forma independente em cada vetor
df_tx = corrigir_wrap_around(df_tx)
df_rx = corrigir_wrap_around(df_rx)

print("--- DIAGNÓSTICO LINEARIZADO ---")
print(f"Primeiro SEQ Absoluto TX: {df_tx['seq_absoluto'].iloc[0]} | Último: {df_tx['seq_absoluto'].iloc[-1]}")
print(f"Primeiro SEQ Absoluto RX: {df_rx['seq_absoluto'].iloc[0]} | Último: {df_rx['seq_absoluto'].iloc[-1]}")

# 3. Cruzar dados usando o ID absoluto imune a repetições
df = pd.merge(df_tx, df_rx, on='seq_absoluto')
print(f"Pacotes em comum (Match): {len(df)}")
print("-------------------\n")

if len(df) > 0:
    df['latency'] = (df['time_rx'] - df['time_tx']) * 1000 # ms
    print("Estatísticas da Latência (ms):")
    print(df['latency'].describe())
else:
    print("ERRO: Nenhum pacote coincidente após linearização.")