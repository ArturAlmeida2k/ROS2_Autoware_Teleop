# Nuvem de pontos quantizada e comprimida para transporte em links limitados.
# O campo data é auto-descritivo: ver o cabeçalho PCQ1 em pcq_codec.py

std_msgs/Header header

uint32 num_points     # nº de pontos após filtragem
string format         # identificador do codec, ex: "PCQ1"
uint8[] data          # blob comprimido
