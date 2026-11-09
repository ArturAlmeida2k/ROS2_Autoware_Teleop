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
  `/teleop/safe_command` and, while the car is in REMOTE, computes the acceleration (P controller
  on the speed) and the steering wheel torque (PD controller on the steering wheel angle). It
  publishes `/teleop/vehicle_command`, with the engage request, and the latency metrics.
- `vehicle_interface` takes the role Autoware had. It owns the modes and publishes
  `/teleop/vehicle_mode`, and runs at a fixed rate (100 Hz) between `control` and the car: it
  sends the last command to `/control/desired_acceleration` (-2 to 1 m/s2) and
  `/control/desired_steering_torque` (-2 to 2 Nm), clamps them to the car limits, holds the car
  braked in STOP, brakes if `control` stops sending, and publishes `/control/indicated_speed` (km/h).
- `vehicle_telemetry` builds `/teleop/telemetry` from `/vehicle/wheel_speed_*`, `/vehicle/gear`,
  `/vehicle/blinkers` and `/vehicle/direction`.

Modes: LOCAL at start-up and after the driver presses the brake pedal, REMOTE after engage, STOP
after disengage. Outside REMOTE the car is always sent a braking command and zero steering torque.
The car ignores them while the driver has it, and when the driver activates PC control again the
car stays stopped instead of using the last command, so the operator can engage.
The engage only acts when it changes, so after the driver takes over the operator has to
disengage and engage again. The car interface has no gear or turn signal commands, so these
are not sent. The gear is chosen by the driver in the car, and the station can no longer change
it (the gear buttons were removed from the controller nodes and the Command Gate). The gear is
still shown in the interface, from the telemetry.

Cameras are set in `vh_bringup/config/vehicle.yaml`. Parameters in
`vh_teleop_to_vehicle/config/vehicle_params.yaml` (steering ratio, steering sign and gains) are
estimates and must be checked on the car.

The simulation setup (Autoware, AWSIM, CARLA, webcam) is in the `main` branch.
