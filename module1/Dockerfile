# Use Ubuntu 22.04 as base image
FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep2 \
    python3-vcstool \
    build-essential \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set locale
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Add ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor | tee /usr/share/keyrings/ros-archive-keyring.gpg > /dev/null

# Add ROS 2 repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Iron
RUN apt-get update && apt-get install -y \
    ros-iron-desktop \
    ros-iron-gazebo-ros-pkgs \
    ros-iron-gazebo-ros2-control \
    ros-iron-xacro \
    ros-iron-joint-state-publisher \
    ros-iron-robot-state-publisher \
    ros-iron-ros2-control \
    ros-iron-ros2-controllers \
    ros-iron-hardware-interface \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python dependencies for AI integration
RUN apt-get update && apt-get install -y python3-rosinstall && rm -rf /var/lib/apt/lists/*

# Install Python packages for AI integration
RUN pip3 install --no-cache-dir \
    openai \
    transformers \
    torch \
    requests \
    numpy \
    matplotlib

# Source ROS 2 setup
ENV ROS_DISTRO=iron
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# Create workspace
RUN mkdir -p /workspace/athena_ws/src

# Set working directory
WORKDIR /workspace/athena_ws

# Copy setup scripts if they exist, otherwise create basic structure
RUN mkdir -p src/athena_description \
    src/athena_bringup \
    src/athena_control \
    src/athena_gazebo \
    src/athena_examples

CMD ["/bin/bash", "-c", "source /opt/ros/iron/setup.bash && exec \"$@\"", "--"]