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

#### Install ROS-O

##### 1. Set up the ROS-O apt repository

```bash
# Install curl (if not already installed)
sudo apt install curl

# Download the GPG key
sudo curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg

# Add the apt repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros1.list

# Add debug package repository (commented out)
echo "# deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs) main-dbg" | sudo tee -a /etc/apt/sources.list.d/ros1.list
```

##### 2. Install and configure rosdep

rosdep is a tool for managing ROS package dependencies.

```bash
# Update package list
sudo apt update

# Install rosdep
# Note: Do not install python3-rosdep2 (it is an older version)
sudo apt install python3-rosdep

# Initialize rosdep
sudo rosdep init
```

##### 3. Define custom rosdep mappings for ROS-O

```bash
# Add package mapping for ROS-O
echo "yaml https://ros.packages.techfak.net/ros-one.yaml one" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-one.list

# Update rosdep database
rosdep update
```

##### 4. Install ROS packages

Install the required ROS packages. For example, to install the desktop version:

```bash
sudo apt install ros-one-desktop
```

Other packages can be installed similarly with the `ros-one-` prefix.

##### 5. Environment setup

After installation, enable the ROS environment with the following command:

```bash
source /opt/ros/one/setup.bash
```

#### Build the workspace

```bash
# Create workspace
mkdir -p ~/ros/home_drone/src
cd ~/ros/home_drone/src

# Clone repositories using vcs
vcs import --input https://raw.githubusercontent.com/iory/jsk_aerial_robot/refs/heads/draugas/robots/mini_quadrotor/home_drone.repos

# Install dependencies
cd ~/ros/home_drone
source /opt/ros/one/setup.bash
rosdep install --from-paths src --ignore-src -y -r

# Build
catkin build mini_quadrotor

# Source the workspace
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
