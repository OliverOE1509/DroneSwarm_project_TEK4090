#!/usr/bin/env bash
set -euo pipefail

docker run -it \
  --gpus all \
  --net=host \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  -e DISPLAY=$DISPLAY\
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/.Xauthority:/root/.Xauthority:rw \
  -v "$(pwd)/ros2_ws_2:/usr/local/ros2_ws" \
  webots-drone
