#!/usr/bin/env bash
set -e
cd ./ros2_ws/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch mavic_simulation robot_launch.py

