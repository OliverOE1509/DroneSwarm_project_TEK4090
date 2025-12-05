ARG BASE_IMAGE=nvidia/cuda:11.8.0-base-ubuntu22.04

###############################################
# Stage 1: Download Webots tarball
###############################################
FROM ${BASE_IMAGE} AS downloader

ARG WEBOTS_VERSION=R2025a
ARG WEBOTS_PACKAGE_PREFIX=
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y wget bzip2 && \
    rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/cyberbotics/webots/releases/download/$WEBOTS_VERSION/webots-$WEBOTS_VERSION-x86-64$WEBOTS_PACKAGE_PREFIX.tar.bz2 && \
    tar xjf webots-*.tar.bz2 && \
    rm webots-*.tar.bz2


###############################################
# Stage 2: Main image
###############################################
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

##############
# Dependencies
##############
RUN apt-get update && \
    apt-get install -y \
        wget xvfb locales vim curl gnupg2 lsb-release \
        python3 python3-pip \
        python3-colcon-common-extensions \
        python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential && \
    rm -rf /var/lib/apt/lists/*

##############
# Install Webots dependencies
##############
RUN wget https://raw.githubusercontent.com/cyberbotics/webots/master/scripts/install/linux_runtime_dependencies.sh && \
    chmod +x linux_runtime_dependencies.sh && \
    ./linux_runtime_dependencies.sh && \
    rm linux_runtime_dependencies.sh

##############
# Install Webots itself
##############
WORKDIR /usr/local/
COPY --from=downloader /webots /usr/local/webots/
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH="${WEBOTS_HOME}:${PATH}"
ENV QTWEBENGINE_DISABLE_SANDBOX=1

##############
# Install ROS 2 Humble
##############
RUN apt-get update && \
    apt-get install -y software-properties-common curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    apt-get install -y ros-humble-desktop python3-argcomplete && \
    rm -rf /var/lib/apt/lists/*

##############
# Install webots_ros2
##############
RUN bash -c "source /opt/ros/humble/setup.bash && \
    apt-get update && apt-get install -y 'ros-humble-webots-ros2*' && \
    rm -rf /var/lib/apt/lists/*"

##############
# Set locales
##############
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

##############
# rosdep setup
##############
RUN rosdep init || true
RUN rosdep update

##############
# Enable NVIDIA OpenGL
##############
ENV NVIDIA_DRIVER_CAPABILITIES="graphics,compute,utility"

##############
# Entry point for environment setup
##############
COPY <<EOF /ros_entrypoint.sh
#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export WEBOTS_HOME=/usr/local/webots
export PATH=/usr/local/webots:\$PATH
exec "\$@"
EOF

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]
