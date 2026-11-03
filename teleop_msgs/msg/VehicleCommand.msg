# Comando do nó control para o nó vehicle_interface, já nas unidades do carro
std_msgs/Header header
float32 acceleration     # m/s2, positivo acelera, negativo trava
float32 steering_torque  # Nm no volante
bool    engage           # pedido do operador (o vehicle_interface decide o modo)
