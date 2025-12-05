# Webots-Docker

[![Dockerhub](https://img.shields.io/docker/automated/cyberbotics/webots.svg)](https://hub.docker.com/r/cyberbotics/webots)
[![Test](https://github.com/cyberbotics/webots-docker/workflows/Test/badge.svg)](https://github.com/cyberbotics/webots-docker/actions?query=workflow%3ATest)
[![Docker Image CI](https://github.com/cyberbotics/webots-docker/workflows/Docker%20Image%20CI/badge.svg)](https://github.com/cyberbotics/webots-docker/actions?query=workflow%3A%22Docker+Image+CI%22)

This is a cloned repository of: https://github.com/cyberbotics/webots-docker and https://github.com/patrickpbarroso/drone-simulation/tree/main 
The purpose of this repo is to provide simulation environment for Webots R2025a, using Docker in a ubuntu 22.04 environment. 


The first you should do if your not on a linux system, is to [install virtual box](https://www.virtualbox.org/wiki/Downloads) and [download a ubuntu image](https://ubuntu.com/download/desktop) to mount on a virtual machine for example. Then you have to [download docker](https://cyberbotics.com/doc/guide/installation-procedure?tab-os=macos#installing-the-docker-image)

The purpose of this repo is to provide simulation environment for Webots R2025a, using Docker in a ubuntu 22.04 environment. In this environment we will run a swarm simulation using webots as the simulation engine and ROS2 to implement the logic and hardware internals of our drone. The goal of the swarm will be to play capture the flag

``` bash
git clone https://github.com/OliverOE1509/DroneSwarm_project_TEK4090.git
cd DroneSwarm_project_TEK4090
```

# Directory structure
Here is the overall directory structure (this will change)
``` bash
.
├── Dockerfile
├── README.md
└── ros2_ws
    ├── Webots-R2025a.conf
    └── src
        └── drone_swarm_ctf_package
            ├── controllers
            ├── package.xml
            └── worlds
```


6 directories, 9 files

# Important files
Will give more details later

``` bash
./Dockerfile
```

This is the dockerfile you build, which opens to container. Dismiss the Dockerfiles "Dockerfile_ikpy" and "Dockerfile_webots_cloud"


## Build the Image
``` bash
docker build -t webots-drone .
```

Now you need to run this command to allow launch files to open up the webots simulation
``` bash
xhost +local:docker
```

Now just simply run 
``` bash
./run.sh test_fra_github
```
And you are now inside the container. Copy these commands to build the workspace
``` bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install
```
Now to start simulation, run 
``` bash
ros2 launch mavic_simulation robot_launch.py
```
You will be asked of how many drones to simulate. Enter a number and webots should open up
