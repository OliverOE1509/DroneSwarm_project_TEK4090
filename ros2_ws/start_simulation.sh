#!/usr/bin/env bash
set -e
#cd ./ros2_ws/
NDrones="${1:-4}"
clear
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build
ros2 launch mavic_simulation robot_launch.py NDrones:=${NDrones} 
#nohup ros2 launch mavic_simulation robot_launch.py > output.log 2>&1 &
