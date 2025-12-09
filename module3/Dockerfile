# Isaac ROS 2 Dockerfile
# Dockerfile.isaac.ros

FROM nvcr.io/nvidia/isaac-ros:ros2-humble-isaac-ros-2.2.0

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute

# Install additional dependencies for humanoid robotics
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    rsl-rl==1.0.2 \
    torch==2.3.0 \
    numpy==1.24.3 \
    onnx==1.15.0 \
    onnxruntime==1.17.0

# Copy module-specific configurations
COPY athena_config.yaml /etc/isaac-ros/athena_config.yaml

# Create workspace directory
RUN mkdir -p /workspaces/isaac_ros_ws/src

WORKDIR /workspaces/isaac_ros_ws

# Source ROS environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /opt/isaac-ros-dev/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]