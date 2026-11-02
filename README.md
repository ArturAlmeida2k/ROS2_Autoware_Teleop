# ROS2_Autoware_Teleop

## Vehicle side (branch vehicle-direct)

In this branch the vehicle side talks to the car directly, without Autoware or a simulator:

```bash
colcon build
ros2 launch vh_bringup vh_launch.py ip_address:=<remote station IP>
```

The launch starts the network nodes (`input_teleop_decoder`, `telemetry_encoder`, `video_encoder`),
the link safety nodes (`topic_monitor`, `safety_gate`) and the two nodes of `vh_teleop_to_vehicle`:

- `vehicle_control` reads `/teleop/safe_command` and the car state and publishes
  `/control/desired_acceleration` (-2 to 1 m/s2), `/control/desired_steering_torque` (-2 to 2 Nm)
  and `/control/indicated_speed` (km/h). Steering is a PD controller on the steering wheel angle.
- `vehicle_telemetry` builds `/teleop/telemetry` from `/vehicle/wheel_speed_*`, `/vehicle/gear`,
  `/vehicle/blinkers` and `/vehicle/direction`.

Modes: LOCAL at start-up (nothing is published), REMOTE after engage, STOP after disengage
(the car is held braked). Pressing the brake pedal in the car always returns to LOCAL.
The car interface has no gear or turn signal commands, so these are not sent. Throttle is only
applied when the car gear matches the requested gear.

Cameras are set in `vh_bringup/config/vehicle.yaml`. Parameters in
`vh_teleop_to_vehicle/config/vehicle_control.yaml` (steering ratio, steering sign and gains) are
estimates and must be checked on the car.

The simulation setup (Autoware, AWSIM, CARLA, webcam) is in the `main` branch.
