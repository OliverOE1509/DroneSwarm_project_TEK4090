#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install
#ros2 run rqt_control rqt_control