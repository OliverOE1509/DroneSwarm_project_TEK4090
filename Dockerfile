ARG BASE_IMAGE=nvidia/cuda:11.8.0-base-ubuntu22.04
FROM ${BASE_IMAGE}

ENV ROS_DOMAIN_ID=0
ENV ROS_LOCALHOST_ONLY=1
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /usr/local/

#Install dependencies
RUN apt-get update && \
    apt-get install -y software-properties-common curl gnupg2 lsb-release

# Install webots
RUN curl -fsSL https://cyberbotics.com/Cyberbotics.asc \
  | gpg --dearmor -o /etc/apt/keyrings/cyberbotics.gpg &&\
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/cyberbotics.gpg] https://cyberbotics.com/debian binary-amd64/" | tee /etc/apt/sources.list.d/cyberbotics.list &&\
    apt-get update && apt-get install -y webots

ENV WEBOTS_HOME=/usr/local/webots

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y ros-humble-desktop python3-argcomplete && \
    rm -rf /var/lib/apt/lists/* 

# Install webots-ros2
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
   apt-get update && apt-get install -y 'ros-humble-webots-ros2*'"  

# Install colcon
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions locales \
    && rm -rf /var/lib/apt/lists/*

# Install rosdep and build tools
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true
RUN rosdep update

# Enable OpenGL capabilities
ENV NVIDIA_DRIVER_CAPABILITIES graphics,compute,utility

# Set user name
ENV USER root

# Set the locales
RUN locale-gen en_US.UTF-8
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

# Install python
RUN apt-get update && apt-get install --yes \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/

# Open a terminal
CMD ["/bin/bash"]
