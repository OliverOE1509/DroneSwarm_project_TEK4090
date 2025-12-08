#!/usr/bin/env bash
set -e
cd ./ros2_ws/
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install
ros2 launch mavic_simulation robot_launch.py
#nohup ros2 launch mavic_simulation robot_launch.py > output.log 2>&1 &
