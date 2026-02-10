1º
cd AWSIM-Demo-Lightweight/
./AWSIM-Demo-Lightweight.x86_64

2º
source  autoware/install/setup.bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle ...

3º
source  autoware/install/setup.bash
(optional) ros2 run joy joy_enumerate_decives #to check id of logitec
ros2 launch manual_teleop G923_launch.py device_id:=1

For degub
source  autoware/install/setup.bash
ros2 topic echo /external/selected/control_cmd