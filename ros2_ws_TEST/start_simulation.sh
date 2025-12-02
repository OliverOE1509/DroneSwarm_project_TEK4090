#!/usr/bin/env bash
set -e

# Activate ROS 2 Humble
source /opt/ros/humble/setup.bash

echo "Sourcing ROS 2 Humble setup.bash"

# Go to workspace root (directory where this script lives)
cd /usr/local/ros2_ws

# Run rosdep (installs missing dependencies)
rosdep install --from-paths src -r -y

echo "Installed dependencies with rosdep"

# Build workspace
colcon build --symlink-install

echo "Built ROS 2 workspace"

# Source overlay
source install/local_setup.bash

echo "Sourced ROS 2 workspace overlay"

echo "Testing Python imports..."
python3 - << 'EOF'
try:
    import drone_swarm_ctf_package
    from drone_swarm_ctf_package import drone1_driver, drone2_driver
    print("[OK] Import succeeded:")
    print("     Drone1Driver:", drone1_driver.Drone1Driver)
    print("     Drone2Driver:", drone2_driver.Drone2Driver)
except Exception as e:
    print("[FAIL] Import failed:", e)
    raise
EOF
echo "Python import test passed."

# Launch the Webots simulation with two drones
ros2 launch drone_swarm_ctf_package two_drones_webots.launch.py
echo "Launched two_drones_webots.launch.py"