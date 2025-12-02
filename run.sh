#!/usr/bin/env bash

set -euo pipefail
# Activate ROS 2 Humble
docker run -it   --gpus all   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v $(pwd)/ros2_ws_TEST/:/usr/local/ros2_ws   webots-drone
