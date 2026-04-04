#!/usr/bin/env bash
set -e

# Load ROS2
source /opt/ros/jazzy/setup.bash

# Load your workspace
source /home/lawnbot/mower_ws/install/setup.bash

# Run node (replace with your real password)
exec ros2 run rtk_reader rtk_reader --ros-args -p password:=none

