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
rosrun gimbalrotor_remote remote_rviz.sh <robot host>
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
- `CameraCloud` (`rviz/DepthCloud`, on, in `arm.rviz` and `room.rviz`): the D435 cloud, made by rviz on
  this PC from the compressed images of the robot (`compressedDepth` with RVL for the depth, `compressed`
  JPEG for the color). The robot compresses and sends them only while the display is enabled; measured on
  the real machine it costs about 18 % of one onboard core and about 1.4 MB/s at the rates below.
- `FastLioMap` (`/cloud_registered`, off): the map of fast_lio. Its frame `camera_init` is where the robot
  started, and nothing connects it to the fixed frame yet, so enable it only with such a transform.

## camera images and cloud on this PC

The robot sends only compressed images: `bringup.launch` sets the depth to RVL and the color to JPEG
(quality 60) and does not make the cloud on board. Nodes that need raw images or a `PointCloud2`, e.g.
grape_detector, run on this PC on the images decompressed here:

```bash
roslaunch gimbalrotor_remote camera_remote.launch       # /gimbalrotor/camera_remote/...
roslaunch grape_detector grape_detection.launch camera_ns:=gimbalrotor/camera_remote
```

`/gimbalrotor/camera_remote/depth/color/points` is the colored cloud. Unlike the rviz display, the
republishers of this launch subscribe for as long as it runs.

Measured on the real machine (Khadas VIM4, one 640x480 frame):

| | onboard encoding | size |
| --- | --- | --- |
| depth, PNG level 1 (the default of compressedDepth) | 20.5 ms | 87 KB |
| depth, RVL | 3.7 ms | 191 KB |
| color, JPEG quality 60 | 3.7 ms | 19 KB |

RVL is lossless like PNG and takes about a fifth of its CPU, for about twice the size. With the D435 on
a USB 2 port the aligned depth comes at about 4.5 Hz (0.8 MB/s as RVL) and the color at about 26 Hz
(0.5 MB/s); on USB 3 the depth reaches 30 Hz, about 5.7 MB/s as RVL, so lower `depth_fps` there if the
network is tight.

## config/room.rviz

The other config is for flying in a room whose map was recorded before: it shows the map of
`gimbalrotor/script/room_localization.py` (`/room_localization/map_cloud`) in the `map` frame, the robot
model and the flight teleop panel, and it has the "2D Pose Estimate" tool of rviz. Dragging that tool on the
robot tells room_localization where the robot is, which is how a wrong match (a rectangular room looks the
same turned by 180 deg) is corrected, as in a 2D localization.

```bash
roslaunch gimbalrotor_remote remote_rviz.launch rviz_config:=$(rospack find gimbalrotor_remote)/config/room.rviz
```

The robot model appears once the match has published `map -> world`.

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
