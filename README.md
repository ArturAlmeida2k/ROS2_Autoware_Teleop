# ROS2_Autoware_Teleop

## Vehicle side (branch vehicle-direct)

In this branch the vehicle side talks to the car directly, without Autoware or a simulator:

```bash
colcon build
ros2 launch vh_bringup vh_launch.py ip_address:=<remote station IP>
```

The launch starts the network nodes (`input_teleop_decoder`, `telemetry_encoder`, `video_encoder`),
the link safety nodes (`topic_monitor`, `safety_gate`) and the nodes of `vh_teleop_to_vehicle`:

```
safety_gate -> control -> vehicle_interface -> car
```

- `control` has the driving logic, like the Autoware version. It reacts to each
  `/teleop/safe_command`, manages the modes, computes the acceleration (P controller on the speed)
  and the steering wheel torque (PD controller on the steering wheel angle), and publishes
  `/teleop/vehicle_command` and the latency metrics.
- `vehicle_interface` sits between `control` and the car and runs at a fixed rate (100 Hz). It sends
  the last command to `/control/desired_acceleration` (-2 to 1 m/s2) and
  `/control/desired_steering_torque` (-2 to 2 Nm), clamps them to the car limits, brakes if
  `control` stops sending, sends nothing while the brake pedal is pressed, and publishes
  `/control/indicated_speed` (km/h).
- `vehicle_telemetry` builds `/teleop/telemetry` from `/vehicle/wheel_speed_*`, `/vehicle/gear`,
  `/vehicle/blinkers` and `/vehicle/direction`.

Modes: LOCAL at start-up (nothing is sent to the car), REMOTE after engage, STOP after disengage
(the car is held braked). Pressing the brake pedal in the car always returns to LOCAL.
The engage only acts when it changes, so after the driver takes over the operator has to
disengage and engage again. The car interface has no gear or turn signal commands, so these
are not sent. Throttle is only applied when the car gear matches the requested gear.

Cameras are set in `vh_bringup/config/vehicle.yaml`. Parameters in
`vh_teleop_to_vehicle/config/vehicle_params.yaml` (steering ratio, steering sign and gains) are
estimates and must be checked on the car.

The simulation setup (Autoware, AWSIM, CARLA, webcam) is in the `main` branch.
