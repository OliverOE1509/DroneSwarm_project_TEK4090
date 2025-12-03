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

  oliver_mac)

    # ensure XAUTHORITY is set
    : "${XAUTHORITY:=$HOME/.Xauthority}"
    touch "$XAUTHORITY"
    
    docker run -it \
      --gpus all \
      -e DISPLAY="$DISPLAY" \
      -e XAUTHORITY="$XAUTHORITY" \
      -v "$XAUTHORITY:$XAUTHORITY" \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      webots-drone
    ;;

  *)
    echo "Invalid argument."
    echo "Run './run.sh d' for the default setup."
    echo "Run './run.sh oliver_mac' for macOS/XQuartz setup."
    exit 1
    ;;
esac
