# ROS2_Autoware_Teleop

Teleoperation framework for an autonomous vehicle over a 5G link, built on ROS 2 Humble and Autoware 1.8.0.
The remote station sends driving commands to the vehicle, and the vehicle sends back telemetry, video and,
optionally, the point cloud. The framework was developed as part of an MSc thesis at the University of Aveiro
(Instituto de Telecomunicações).

## Branches

- **`main`** (this branch): full setup with Autoware. The vehicle side connects to Autoware through its
  standard interfaces, so it can run in AWSIM, CARLA or with a real vehicle.
- **`vehicle-direct`**: vehicle side that talks directly to the drive-by-wire interface of the car, without
  Autoware or a simulator. See the README of that branch for details.

## Structure

| Folder | Role |
|---|---|
| `rs_manual_teleop/` | Remote station: controller nodes (Logitech RS50, G923, Xbox), command gate, network encoders and decoders, and the operator GUI (`rs_gui`) |
| `vh_manual_teleop/` | Vehicle side: network nodes, link safety, Autoware bridge (`vh_teleop_to_autoware`), telemetry and launch files |
| `vh_camera/` | Webcam publisher, used as a camera source for tests |
| `teleop_msgs/` | Custom messages (commands, telemetry, metrics) and shared enums |

## Requirements

- Ubuntu 22.04 with ROS 2 Humble
- Autoware 1.8.0 (for the `main` setup)
- AWSIM or CARLA for simulated tests
- Python dependencies for the network nodes: `vh_manual_teleop/vh_network/requirements.txt`

## Build

```bash
cd <workspace>
colcon build
source install/setup.bash
```

## Run

Vehicle side (replace the IP with the address of the remote station):

```bash
ros2 launch vh_bringup vh_launch.py ip_address:=<remote station IP>
```

Remote station:

```bash
ros2 launch rs_bringup rs_launch.py
```

Ports used by the vehicle side, with the defaults of the launch files:

| Flow | Transport | Port |
|---|---|---|
| Commands (station to vehicle) | UDP | 5005 |
| Telemetry (vehicle to station) | UDP | 5006 |
| Video (one port per camera, `camera_port + i`) | RTP over UDP | 5007 onwards |
| Point cloud | TCP | `pointcloud_port` |

Each node reads its settings from the YAML files in `vh_bringup/config/` (`awsim.yaml`, `carla.yaml`,
`webcam.yaml`), so the simulator or camera setup can be changed without editing the code.

## Safety

- The remote station only forwards commands while the operator has engaged the vehicle and the link is healthy.
  Without telemetry for 3 s, the command gate stops sending commands.
- The vehicle side monitors the link latency and changes state (OK, WARN, ERROR) with a hysteresis of 5 cycles.
  In ERROR the vehicle brakes fully and turns on the hazard lights.

## Thesis

The architecture, the measurements and the limitations are described in the thesis
*Teleoperation of an Autoware-based Autonomous Vehicle* (Artur Almeida, 2026).
