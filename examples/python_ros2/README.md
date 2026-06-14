# Example: Python (rclpy) user controller

A minimal controller that talks to the bridge over local ROS2.

## 1. Start the bridge (in user_repo_new)

```bash
cd ../..
./start.sh ABC123 --ros-domain-id 0
```

This connects to the simulation and exposes, for your vessel:
- `/<vessel>/vessel_state` (telemetry, `std_msgs/Float64MultiArray`)
- `/<vessel>/actuator_cmd` (commands, `interfaces/Actuator`)

## 2. Build the message package (once)

See [`../../interfaces/README.md`](../../interfaces/README.md):

```bash
mkdir -p ~/mavsim_ws/src && cp -r ../../interfaces ~/mavsim_ws/src/
cd ~/mavsim_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select interfaces
source ~/mavsim_ws/install/setup.bash
```

## 3. Run your controller

```bash
export ROS_DOMAIN_ID=0          # MUST match the bridge's --ros-domain-id
python3 example_controller.py --vessel matsya_01 --cs cs_01 --th th_01
```

Find your vessel's `ros_name` and actuator names in the simulation UI (or the
bridge logs print the vessel it bound and the `/<vessel>/actuator_cmd` topic).

## Sanity checks (separate terminal, same ROS_DOMAIN_ID)

```bash
ros2 topic list                                   # should show /<vessel>/...
ros2 topic echo /matsya_01/vessel_state           # telemetry flowing?
ros2 topic echo /matsya_01/actuator_cmd           # your commands flowing?
```

## Notes

- If the simulation uses quaternion attitude, pass `--quaternion`.
- The bridge stops forwarding your command if none arrives within
  `--cmd-timeout` seconds (default 1s), so publish continuously.
- This example controls one vessel. To control several from one process, create
  one publisher/subscriber pair per vessel `ros_name` you own.
