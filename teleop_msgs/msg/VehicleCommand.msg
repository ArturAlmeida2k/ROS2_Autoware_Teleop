# Comando do nó control para o nó vehicle_interface
std_msgs/Header header
float32 acceleration         # m/s2, positivo acelera, negativo trava
float32 steering_tire_angle  # rad, ângulo das rodas
bool    engage               # true = o operador quer conduzir (REMOTE)
uint8   gear                 # mudança pedida (CommandEnums GEAR_*)
