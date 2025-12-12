#!/usr/bin/env bash
set -e
cd ./ros2_ws/
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install
nohup ros2 launch mavic_simulation robot_launch.py > output.log 2>&1 &
