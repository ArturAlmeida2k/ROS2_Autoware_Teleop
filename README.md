# ROS2_Autoware_Teleop

## Direct vehicle interface (no Autoware)

The vehicle side can talk to the car directly instead of going through Autoware:

```bash
ros2 launch vh_bringup vh_launch.py target:=vehicle sim:=vehicle ip_address:=<remote station IP>
```

`target:=vehicle` replaces `control` and `telemetry_node` with the nodes of `vh_teleop_to_vehicle`:

- `vehicle_control` reads `/teleop/safe_command` and the car state and publishes
  `/control/desired_acceleration` (-2 to 1 m/s2), `/control/desired_steering_torque` (-2 to 2 Nm)
  and `/control/indicated_speed` (km/h). Steering is a PD controller on the steering wheel angle.
- `vehicle_telemetry` builds `/teleop/telemetry` from `/vehicle/wheel_speed_*`, `/vehicle/gear`,
  `/vehicle/blinkers` and `/vehicle/direction`.

Modes: LOCAL at start-up (nothing is published), REMOTE after engage, STOP after disengage
(the car is held braked). Pressing the brake pedal in the car always returns to LOCAL.
The car interface has no gear or turn signal commands, so these are not sent. Throttle is only
applied when the car gear matches the requested gear.

Parameters in `vh_teleop_to_vehicle/config/vehicle_control.yaml` (steering ratio, steering sign
and gains) are estimates and must be checked on the car.

Without Autoware, build only the packages that do not need it:

```bash
colcon build --packages-skip vh_teleop_to_autoware vh_telemetry
```
