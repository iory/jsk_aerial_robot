# gimbalrotor_remote

rviz panels to operate [gimbalrotor](../gimbalrotor/README.md) from an operator PC: the arm (servo torque
and joint targets) and the basic flight teleoperation.

## TL;DR

```bash
source ~/ros/grape/devel/setup.bash

# same PC as the robot or the simulation (prefix with QT_QPA_PLATFORM=xcb under Wayland)
roslaunch gimbalrotor_remote remote_rviz.launch

# operator PC connected to the robot: sets ROS_MASTER_URI / ROS_IP, then launches the same file
rosrun gimbalrotor_remote remote_rviz.sh <robot host>
```

`ArmControlPanel` turns the servo torque of each arm joint on and off and moves the joints in degrees;
`FlightTeleopPanel` arms, takes off, lands, halts and moves the robot while a direction button is held.
**On the real machine those flight buttons spin the propellers.**

## launch

On the PC that also runs the robot or the simulation:

```bash
roslaunch gimbalrotor_remote remote_rviz.launch
```

On an operator PC connected to the ROS master of the robot:

```bash
rosrun gimbalrotor_remote remote_rviz.sh <robot host>   # e.g. 192.168.97.101
```

`remote_rviz.sh` resolves the robot address, exports `ROS_MASTER_URI` and `ROS_IP` (the source address of
the route to the robot), sets `QT_QPA_PLATFORM=xcb` for Wayland sessions and then runs `remote_rviz.launch`.
The nodes on the robot must advertise an address this PC can reach, i.e. start the robot launch with
`ROS_IP=<robot ip>` exported; the script warns when a node address is not resolvable here.

Arguments of `remote_rviz.launch`: `robot_ns` (default `gimbalrotor`), `arm_controller_ns`
(`arm/arm_controller`), `rviz_config` (`config/arm.rviz`). It also starts
`gimbalrotor/arm_torque_server.py`, which the arm panel calls.

## displays

Besides the robot model, `config/arm.rviz` has two point cloud displays:

- `LivoxScan` (`/livox/lidar`, on): the live scan. The driver has to publish PointCloud2 in the frame of the
  robot model, i.e. start it as
  `roslaunch livox_ros_driver2 msg_MID360.launch xfer_format:=0 msg_frame_id:=gimbalrotor/lidar_imu`
  (the default `xfer_format:=1` is a livox CustomMsg, which rviz cannot show). It is about 3.8 MB/s over the
  network, so turn it off when it is not needed.
- `FastLioMap` (`/cloud_registered`, off): the map of fast_lio. Its frame `camera_init` is where the robot
  started, and nothing connects it to the fixed frame yet, so enable it only with such a transform.

## ArmControlPanel

Per joint of `arm_controller`: servo torque state (ON green / OFF red / `?` when
`<robot_ns>/servo/torque_states` is not received), ON and OFF buttons, the measured angle and a target
slider and spin box in degrees, limited by the joint limits of the URDF.

- `Target <- current` sets the targets to the measured angles, `Target <- init pose (0 deg)` sets them to 0.
- `Move` sends the targets as one `FollowJointTrajectory` goal over `duration`.
- `move on slider release` sends them as soon as a slider is released (off by default).
- The targets follow the measured angles when the panel loads and after every torque command, so `Move`
  never sends a stale target.

## FlightTeleopPanel

Same topics as `aerial_robot_base/keyboard_command.py`: `teleop_command/{start,takeoff,land,halt,force_landing}`
and `uav/nav`. It shows the flight state, the position and the battery voltage.

- `Arm (start)` and `Takeoff` are enabled only while `enable arming / takeoff` is checked; the check is not
  restored from the rviz config. `Land`, `HALT` and `Force landing` are always enabled.
- The direction buttons move the robot while they are held down, in the world or the body frame, with the
  velocities of the panel. They are enabled only in `HOVER`, the only state in which the navigator accepts
  `uav/nav`, and a released axis gets a zero velocity at once.
- **The buttons spin the propellers of the real machine.**
