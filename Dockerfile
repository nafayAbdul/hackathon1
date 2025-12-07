# Use ROS 2 Iron (Ubuntu 22.04)
FROM ros:iron-ros-base-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=iron
ENV COLCON_WS=/workspace

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    curl \
    wget \
    gnupg \
    lsb-release \
    build-essential \
    locales \
    && locale-gen en_US.UTF-8 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Set locale
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install additional Python packages for AI integration
RUN pip3 install --upgrade pip && \
    pip3 install \
    transformers \
    torch \
    openai \
    numpy \
    matplotlib \
    scipy

# Create workspace
RUN mkdir -p ${COLCON_WS}/src
WORKDIR ${COLCON_WS}

# Copy the project files
COPY module1/ ${COLCON_WS}/src/athena_module1/
COPY specs/ ${COLCON_WS}/specs/
COPY exercises/ ${COLCON_WS}/exercises/
COPY docs/ ${COLCON_WS}/docs/

# Build the ROS 2 packages
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    cd ${COLCON_WS} && \
    colcon build --packages-select athena_description athena_bringup athena_control athena_gazebo --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc

CMD ["bash"]