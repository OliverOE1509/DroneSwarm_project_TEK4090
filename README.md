# Webots-Docker

[![Dockerhub](https://img.shields.io/docker/automated/cyberbotics/webots.svg)](https://hub.docker.com/r/cyberbotics/webots)
[![Test](https://github.com/cyberbotics/webots-docker/workflows/Test/badge.svg)](https://github.com/cyberbotics/webots-docker/actions?query=workflow%3ATest)
[![Docker Image CI](https://github.com/cyberbotics/webots-docker/workflows/Docker%20Image%20CI/badge.svg)](https://github.com/cyberbotics/webots-docker/actions?query=workflow%3A%22Docker+Image+CI%22)

This is a cloned repository of: https://github.com/cyberbotics/webots-docker
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
``` bash
drone_swarm_ctf/worlds//
└── mavic_2_pro.wbt
```

This is the world file you open when starting the simulation. It has a drone already spawned
``` bash
drone_swarm_ctf/controllers/mavic_controller//
└── mavic_controller.py
```

This is the controller to said drone already spawned inside the world file

``` bash
./Dockerfile
```

This is the dockerfile you build, which opens to container. Dismiss the Dockerfiles "Dockerfile_ikpy" and "Dockerfile_webots_cloud"


## Build the Image
``` bash
docker build -t webots-drone .
```

## Run the container from the image 
``` bash
docker run -it   --gpus all   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v $(pwd)/ros2_ws/:/usr/local/ros2_ws   webots-drone
```

If you dont have an nvidia gpu, you can dismiss this from the command above
``` bash
--gpus=all
```


Also, if you want to open webots inside  the container to get an interface to see the simulation, then you need to pass access for the container to use your screen. This is done with
``` bash
-v /tmp/.X11-unix:/tmp/.X11-unix:rw
```

If you are windows, I recommend to use a virtual machine and keep the settings just mentioned. If you are macos, you (WILL ADD LATER)

