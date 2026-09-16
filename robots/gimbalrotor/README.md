# gimbalrotor

Multirotor whose rotors are tilted by gimbal servos. The `grape_with_arm` airframe carries a 5-DoF arm
with a gripper, controlled through `FollowJointTrajectory`.

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

```bash
roslaunch gimbalrotor bringup.launch airframe:=grape_with_arm
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
