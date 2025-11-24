# Webots-Docker

[![Dockerhub](https://img.shields.io/docker/automated/cyberbotics/webots.svg)](https://hub.docker.com/r/cyberbotics/webots)
[![Test](https://github.com/cyberbotics/webots-docker/workflows/Test/badge.svg)](https://github.com/cyberbotics/webots-docker/actions?query=workflow%3ATest)
[![Docker Image CI](https://github.com/cyberbotics/webots-docker/workflows/Docker%20Image%20CI/badge.svg)](https://github.com/cyberbotics/webots-docker/actions?query=workflow%3A%22Docker+Image+CI%22)

This is a cloned repository of: https://github.com/cyberbotics/webots-docker

The purpose of this repo is to provide simulation environment for Webots R2025a, using Docker. To use the docker container clone the repo

''' bash
git clone https://github.com/OliverOE1509/DroneSwarm_project_TEK4090.git
cd DroneSwarm_project_TEK4090
'''

If you want GPU acceleration, just pass this  
''' bash
--gpus all
'''
into the run command. To do this you need to install '''nvidia-docker2''' from the [official website](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

For å builde docker filen kjører jeg
``` bash
docker build -t webots-drone .
```


Jeg åpner hele kontaineren på denne måten. Dette skal du kjøre når du har /path/to/dir/DroneSwarm_Project_TEK4090 som working directory 
``` bash
docker run -it   --gpus all   -e DISPLAY=$DISPLAY   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v $(pwd)/drone_swarm_ctf:/project   webots-drone
```


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




