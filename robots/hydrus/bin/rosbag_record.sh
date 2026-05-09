#!/bin/bash

if [ $# -ne 1 ];then
    echo "Usage: rosrun gimbalrotor rosbag_record.sh file_name_prefix";
    exit 1
fi

rosbag record -a -x ".*/cloud_registered.*" -o $1