#!/usr/bin/env bash
set -e
cd ./ros2_ws/
source install/local_setup.bash
colcon build --symlink-install
ros2 launch mavic_simulation robot_launch.py

