# Drone Swarm Simulation with APF Controller
This repository contains a simulation environment for multiple DJI Mavic 2 Pro drones operating in a Webots world. The drones utilize an Artificial Potential Field (APF) controller to navigate through randomly generated waypoints in continuous operation.

[Watch the demo](Example_simulation.mp4)

## Credits
This project extends the following foundational work:
* [Webots Docker](https://github.com/cyberbotics/webots-docker) - Containerized Webots environment
* [Drone Simulation](https://github.com/patrickpbarroso/drone-simulation/tree/main) - Base drone simulation framework

## Version Requirements
* Webots: R2025a
* ROS 2: Humble distribution
* Docker: Latest recommended version
* Hardware: NVIDIA GPU (for accelerated simulation)
* Linux, UNIX or WSL2 host OS. 

## Quick start guide
Clone Repository.
``` bash
git clone https://github.com/OliverOE1509/DroneSwarm_project_TEK4090.git
cd DroneSwarm_project_TEK4090
```
Build Docker Image.
``` bash
docker build -t webots-drone .
```
Configure Display Permissions.
``` bash
xhost +local:docker
```
Launch Simulation Environment.
``` bash
./run.sh
```
Start Simulation.
``` bash
./start_simulation.sh
```

