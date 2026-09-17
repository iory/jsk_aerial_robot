# gimbalrotor

Multirotor whose rotors are tilted by gimbal servos. The `grape_with_arm` airframe carries a 5-DoF arm
with a gripper, controlled through `FollowJointTrajectory`.

## TL;DR

```bash
source ~/ros/grape/devel/setup.bash

# simulation, one terminal each
roslaunch gimbalrotor bringup.launch simulation:=True real_machine:=False airframe:=grape_with_arm
QT_QPA_PLATFORM=xcb roslaunch gimbalrotor_remote remote_rviz.launch   # rviz with the arm and teleop panels

# grape harvest demo instead of the empty world
roslaunch gimbalrotor grape_harvest_sim.launch headless:=True
rosrun gimbalrotor skrobot_grape_harvest_demo.py --bunches 0 -y

# rviz on an operator PC connected to the robot
rosrun gimbalrotor_remote remote_rviz.sh <robot host>
```

- `headless:=False` shows the gazebo GUI; under Wayland prefix it with `QT_QPA_PLATFORM=xcb`.
- After `Ctrl-C`, wait until gzserver is gone (`pgrep -fc gzserver` is 0) before launching again; a leftover
  one makes the next gazebo exit immediately.
- Run your own scikit-robot scripts with `rosrun gimbalrotor <script>` or
  `devel/share/gimbalrotor/venv/bin/python`.

## bringup

### simulation (gazebo)

```bash
roslaunch gimbalrotor bringup.launch simulation:=True real_machine:=False airframe:=grape_with_arm
```

- `headless:=False` shows the gazebo GUI (default: `True`, gzserver only).
  Under a Wayland session, gzclient and rviz need the X11 backend of Qt:
  ```bash
  QT_QPA_PLATFORM=xcb roslaunch gimbalrotor bringup.launch simulation:=True real_machine:=False \
      headless:=False airframe:=grape_with_arm
  ```
- `airframe:=` selects the config directory under `config` (default: `quad`; `grape_with_arm` for the arm).
  The arm controller and its initial pose gate are launched only for `grape_with_arm`.
- `spawn_x/y/z`, `spawn_yaw` place the robot; the default `spawn_z` of `grape_with_arm` (0.33 m) keeps its
  legs above the ground.
- The state estimate is the gazebo ground truth (`sim_estimate_mode:=2`); on the real machine
  `estimate_mode:=1` uses mocap.

### vineyard of the grape harvest demo (gazebo)

```bash
roslaunch gimbalrotor grape_harvest_sim.launch            # headless:=True for gzserver only
rosrun gimbalrotor skrobot_grape_harvest_demo.py
```

It is `bringup.launch` with `airframe:=grape_with_arm` in `worlds/grape_vineyard.world`, plus the node that
stands in for the contact between the fingers and a stem.

### real machine

One launch brings up everything on the onboard PC: the flight controller bridge, the arm, the D435, the
MID360 with fast_lio (`lidar:=`, on with `estimate_mode:=0`), room_localization (with `room_map:=`) and
grape_detector (`grape_detection:=`, on for grape_with_arm; it picks the NPU of the VIM4 by itself).

```bash
roslaunch gimbalrotor bringup.launch estimate_mode:=0 airframe:=grape_with_arm room_map:=/path/to/room.pcd
```

## rviz on an operator PC

`gimbalrotor_remote` brings up rviz with a panel for the arm (servo torque on/off and joint targets) and a
panel for the flight teleoperation. See [gimbalrotor_remote](../gimbalrotor_remote/README.md).

```bash
# same PC as the simulation
roslaunch gimbalrotor_remote remote_rviz.launch
# operator PC connected to the robot
rosrun gimbalrotor_remote remote_rviz.sh <robot host>
```

## sensors of grape_with_arm (onboard PC)

The onboard PC reaches the LiDAR over Ethernet and the depth camera over USB.

| sensor | driver | topics |
| --- | --- | --- |
| Livox MID360 and fast_lio | `launch/include/sensors.launch.xml` of `bringup.launch`, on with `estimate_mode:=0` (`lidar:=`) | `<robot_ns>/livox/lidar` 10 Hz, `<robot_ns>/livox/imu` 200 Hz; fast_lio: `<robot_ns>/cloud_registered`, `<robot_ns>/Odometry_precede` (the position of the state estimation) |
| Intel RealSense D435 | `launch/include/sensors.launch.xml` of `bringup.launch`, `camera:=false` to skip it | `<robot_ns>/camera/color/image_raw`, `.../aligned_depth_to_color/image_raw`, with `compressed` (JPEG) and `compressedDepth` (RVL); gazebo publishes the same topics |

- The LiDAR and the onboard PC must hold the addresses of `livox_ros_driver2/config/MID360_config.json`.
  Otherwise the driver still receives the points but drops them with
  `Storage point data failed, can not get index` and publishes nothing.
- fast_lio runs in the namespace of the robot, as for dragon: the state estimation (`sensor_plugin/vo`)
  subscribes to `<robot_ns>/Odometry_precede`. Started outside it, fast_lio publishes `/Odometry_precede`,
  which nothing reads, and the state estimation gets no position. It uses the IMU of the MID360.
- The fast_lio cloud goes to the operator PC as `<robot_ns>/lio/cloud`, thinned to `lio_cloud_rate:=`
  (2 Hz) and only while someone subscribes.
- The D435 cloud is not made on board: it is made on the operator PC from the compressed images, see
  [gimbalrotor_remote](../gimbalrotor_remote/README.md).
- Connect the D435 to a USB 3 port: on USB 2 (`connected using a 2.1 port` in its log) the aligned depth
  comes at about 4.5 Hz only.
- `Mipi device capability could not be grabbed` in the RealSense log is harmless: it looks for MIPI cameras
  and the D435 is on USB.

## same coordinates on every run (lidar)

fast_lio starts its world at the pose where it was switched on, so a place has different coordinates on
every run. `script/room_localization.py` matches the cloud of the run against a map recorded once and
publishes `map -> camera_init`, after which a place keeps its coordinates. It also publishes
`map -> world`, the frame the flight targets are in: the lidar odometry and the state estimation describe
the same robot, so comparing their two poses of the moment gives the transform between their frames
(`~world_frame:=''` skips it).

```bash
# record the map once, with pcd_save/pcd_save_en of fast_lio on (writes <fast_lio>/PCD/scans.pcd)
roslaunch gimbalrotor bringup.launch estimate_mode:=0 airframe:=grape_with_arm

# every run: bringup starts room_localization with the map
roslaunch gimbalrotor bringup.launch estimate_mode:=0 airframe:=grape_with_arm room_map:=/path/to/scans.pcd
# or on its own, with the lidar driver and fast_lio (in the namespace of the robot) already running
roslaunch gimbalrotor room_localization.launch map:=/path/to/scans.pcd
```

It takes the wall directions of both clouds (the angle whose histograms of the point coordinates are
sharpest) and the shift that correlates them best, so it needs no initial guess. The walls of a rectangular
room repeat every 180 deg, so the turn whose cloud then covers the map best is taken, which the furniture
inside the room decides; `yaw_hint:=` only breaks a tie. `~relocalize` (std_srvs/Empty) redoes the match.

Carrying the robot once around the room gives a map with all the walls in it. Measured on the real machine
with such a map, two runs put the same standing robot within 0.02 m and 1.3 deg of each other, and within
0.08 m and 2.9 deg of where the map says it is.

The map frame is level even though the lidar of grape_with_arm is mounted upside down: the roll and pitch of
the lidar in the robot model (`~base_frame` -> `~lidar_frame`) are taken off the recorded map and the run
cloud, and `map -> camera_init` carries them (roll 180 deg here). The floor of the map (its lowest horizontal
plane) is at z = 0 (`~floor_at_zero`), and the height of the run is matched too, by lining up the floors
and ceilings of the two clouds, so the robot stands on the floor even when the lidar starts at another
height than at the recording (on a desk, in a hand). This assumes the robot stands level when fast_lio
starts, and needs the robot model (robot_description and its tf) to be running.

When the match is wrong, or the room is too symmetric to decide, point at the robot in rviz as in a 2D
localization: `config/room.rviz` of gimbalrotor_remote shows the map (`~map_cloud` of the node) and has the
"2D Pose Estimate" tool. The robot jumps to the pose given there at once, and a couple of seconds later to
where the cloud fits, so pointing at a wrong place is a quick way to see that the localization runs. The
given pose only decides between fits that are equally good (the same room turned by 180 deg), so point
roughly the right way; its position may be anywhere.

```bash
rosrun gimbalrotor_remote remote_rviz.sh <robot host>   # opens config/room.rviz
```

With that transform, the skrobot interface flies to map coordinates:

```python
ri.move_to_map([1.0, 0.0, 1.2])                      # same place on every run
ri.move_to_map([0.0, 0.0, 1.0], yaw=np.radians(45))  # yaw is in the map frame too
ri.map_position()                                    # where the CoG is in the map frame
ri.map_to_world()                                    # the transform itself
```

Recorded maps are site data and are not in this repository.

## arm

The arm joints are `<robot_ns>/arm/arm_controller` (`position_controllers/JointTrajectoryController` through
`arm_hardware_interface`). The gripper is driven by one of its joints, whose mimic joints are the fingers.

### python (scikit-robot)

```python
from gimbalrotor.skrobot_interface import GimbalrotorROSRobotInterface
ri = GimbalrotorROSRobotInterface()       # use_flight=False if the flight stack is not running
ri.robot.arm_joint2_joint1.joint_angle(0.5)
ri.angle_vector(ri.robot.angle_vector(), 3.0)
ri.wait_interpolation()
ri.start_grasp(); ri.stop_grasp()
ri.torque_off(); ri.torque_on()           # servo torque of the arm joints
ri.start(); ri.takeoff(); ri.go_pos(x=0.5); ri.land()
ri.move_to([1.0, 0.0, 1.2])               # world frame of the estimator
ri.move_to_map([1.0, 0.0, 1.2])           # map frame, see below
```

It needs the scikit-robot venv built by catkin_virtualenv, so run it with
`rosrun gimbalrotor <script>` or `devel/share/gimbalrotor/venv/bin/python`.

### servo torque

`servo/torque_enable` (spinal/ServoTorqueCmd) turns the servo torque off; the joints then go limp and
`arm_hardware_interface` stops commanding them (spinal would turn the torque back on with a position
command). `servo/torque_states` reports the state.

In gazebo `script/servo_torque_sim.py` (launched by `bringup.launch` in simulation) provides the same
topics and switches a joint to a damper while its torque is off, so the python API and the services behave
as on the real machine.

`script/arm_torque_server.py` wraps this in `std_srvs/SetBool` services, which the rviz panel uses:

```bash
rosservice call /gimbalrotor/arm_torque/all "data: false"
rosservice call /gimbalrotor/arm_torque/arm_joint2_joint1 "data: true"
```
