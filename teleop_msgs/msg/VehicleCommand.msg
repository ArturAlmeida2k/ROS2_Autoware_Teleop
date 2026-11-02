# Comando do nó control para o nó vehicle_interface, já nas unidades do carro
std_msgs/Header header
float32 acceleration     # m/s2, positivo acelera, negativo trava
float32 steering_torque  # Nm no volante
uint8   mode             # CommandEnums OPERATION_MODE_* (LOCAL = não enviar nada ao carro)
