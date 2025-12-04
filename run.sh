#!/usr/bin/env bash
set -euo pipefail

ARG="${1:-d}"   # default is "d"

case "$ARG" in
  d)
    docker run -it \
      --gpus all \
      -e DISPLAY="$DISPLAY" \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v "$(pwd)/ros2_ws_TEST:/usr/local/ros2_ws" \
      webots-drone
    ;;
  test_fra_github)
    docker run -it \
      --gpus all \
      --net=host \
      --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
      -e DISPLAY=$DISPLAY\
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -v ~/.Xauthority:/root/.Xauthority:rw \
      -v "$(pwd)/ros2_ws_test2:/usr/local/ros2_ws" \
      webots-drone
    ;;

  oliver_mac)

    # ensure XAUTHORITY is set
    : "${XAUTHORITY:=$HOME/.Xauthority}"
    touch "$XAUTHORITY"

    docker run -it \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v "$(pwd)/ros2_ws_TEST:/usr/local/ros2_ws" \
    webots-drone
    ;;

  *)
    echo "Invalid argument."
    echo "Run './run.sh d' for the default setup."
    echo "Run './run.sh test_fra_github' for the test_fra_github setup. This mount containes the proper swarm structure I found on github"
    echo "Run './run.sh oliver_mac' for macOS/XQuartz setup."
    exit 1
    ;;
esac
