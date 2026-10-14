std_msgs/Header header
builtin_interfaces/Time origin_stamp
uint32  id
float32 velocity_kmh
float32 e2e_command_ms
# origem (relógio do RS) do último comando aplicado no VH
builtin_interfaces/Time command_origin_stamp
uint8   gear
uint8   mode
uint8   turn_signal
uint8   network_state
bool    engaged