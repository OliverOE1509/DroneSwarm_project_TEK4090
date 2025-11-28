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

## What is where?

# Directory structure
Here is the overall directory structure (this will change)
``` bash
.
├── Dockerfile
├── Dockerfile_ikpy
├── Dockerfile_webots_cloud
├── drone_swarm_ctf
│   ├── controllers
│   │   ├── drone_controller
│   │   └── mavic_controller
│   └── worlds
│       ├── drone_navigation.wbt
│       └── mavic_2_pro.wbt
├── README.md
├── simple_test.wbt
├── tree.txt
└── Webots-R2025a.conf
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
docker run -it   --gpus all   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v $(pwd)/drone_swarm_ctf:/usr/local/ros2_ws/src   webots-drone
```

If you dont have an nvidia gpu, you can dismiss this from the command above
``` bash
--gpus=all
```



## Ignore everything after this line

This repository is used to create a Docker image with Webots already pre-installed.
To use the already available image please follow the [Webots installation instructions](https://cyberbotics.com/doc/guide/installation-procedure#installing-the-docker-image).


## Build the Image

Use the following command to build the docker container from the Dockerfile:

``` bash
docker build . --file Dockerfile --tag cyberbotics/webots:latest --build-arg WEBOTS_PACKAGE_PREFIX=_ubuntu-22.04
```

## Build the Webots.Cloud Images

Use the following command to build the docker container from the Dockerfile_webots_cloud:

``` bash
docker build . --file Dockerfile_webots_cloud --tag cyberbotics/webots.cloud:latest
```

## Run a Docker container from the Image

You can run the previously built image with:

``` bash
docker run -it cyberbotics/webots:latest /bin/bash
```

## Clean the temporary Images

You can run the following command to remove **all** temporary images:

``` bash
docker system prune
```




