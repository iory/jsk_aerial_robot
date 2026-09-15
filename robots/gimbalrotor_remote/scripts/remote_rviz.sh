#!/bin/bash
# Launch rviz with the arm control panel on this PC, connected to the ROS master of the robot.
#
# usage: rosrun gimbalrotor_remote remote_rviz.sh ROBOT_HOST [roslaunch args...]
#
# ROS_IP of this PC is the source address of the route to the robot. The nodes on the robot must also
# advertise an address this PC can reach (e.g. start them with ROS_IP=<robot ip>); a warning is printed
# otherwise, since rviz would then receive no data.

set -e

if [ $# -lt 1 ]; then
  echo "usage: $(basename "$0") ROBOT_HOST [roslaunch args...]" >&2
  exit 1
fi
robot_host=$1
shift

robot_ip=$(getent ahostsv4 "$robot_host" | awk 'NR == 1 {print $1}')
if [ -z "$robot_ip" ]; then
  echo "can not resolve $robot_host" >&2
  exit 1
fi
local_ip=$(ip -4 route get "$robot_ip" | sed -n 's/.* src \([0-9.]*\).*/\1/p')
if [ -z "$local_ip" ]; then
  echo "no route to $robot_ip" >&2
  exit 1
fi

export ROS_MASTER_URI="http://${robot_ip}:11311"
export ROS_IP="$local_ip"
unset ROS_HOSTNAME
# rviz (Ogre) needs the X11 backend of Qt under Wayland sessions
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"

if ! timeout 3 bash -c "</dev/tcp/${robot_ip}/11311" 2>/dev/null; then
  echo "no ROS master at $ROS_MASTER_URI" >&2
  exit 1
fi
echo "ROS_MASTER_URI=$ROS_MASTER_URI ROS_IP=$ROS_IP"

python3 - <<'EOF'
import socket
from urllib.parse import urlparse

import rosgraph

master = rosgraph.Master('/remote_rviz_check')
publishers, _, _ = master.getSystemState()
nodes = sorted({node for _, names in publishers for node in names})
unreachable = []
for node in nodes:
    host = urlparse(master.lookupNode(node)).hostname
    try:
        socket.gethostbyname(host)
    except OSError:
        unreachable.append('{} ({})'.format(node, host))
if unreachable:
    print('WARNING: this PC can not resolve the address of these nodes, so rviz will not receive their data:\n  '
          + '\n  '.join(unreachable)
          + '\nstart the robot launch with ROS_IP=<robot ip> exported, or add the robot hostname to /etc/hosts')
EOF

exec roslaunch gimbalrotor_remote remote_rviz.launch "$@"
