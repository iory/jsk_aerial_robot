# Mini Quadrotor

# How to use

In one terminal, run

`roslaunch mini_quadrotor bringup.launch real_machine:=false simulation:=True headless:=False`

In another terminal, run

`rosrun aerial_robot_base keyboard_command.py`

In this terminal, input `r` to arm the quadrotor, then input `t` to takeoff. The quadrotor will takeoff and hover.

Input `l` to land the quadrotor.

## Home drone

### Setup

```
source /opt/ros/${ROS_DISTRO}/setup.bash
mkdir -p ~/ros/home_drone/src
cd ~/ros/home_drone/src
vcs import --input https://raw.githubusercontent.com/iory/jsk_aerial_robot/refs/heads/draugas/robots/mini_quadrotor/home_drone.repos
cd ~/ros/home_drone
rosdep install --from-paths . --ignore-src -y -r src
catkin build mini_quadrotor
source ~/ros/home_drone/devel/setup.bash
```

```
roslaunch mini_quadrotor bringup.launch real_machine:=false simulation:=True headless:=False direct_model:=True direct_model_name:=$(rospack find mini_quadrotor)/urdf/draugas.urdf spawn_z:=0.5
```

```
rosrun aerial_robot_base keyboard_command.py
```

### Use real

```
roslaunch mini_quadrotor bringup.launch real_machine:=true simulation:=false headless:=true direct_model:=True direct_model_name:=$(rospack find mini_quadrotor)/urdf/draugas.urdf spawn_z:=0.5 estimate_mode:=0
```
