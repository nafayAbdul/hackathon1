# Quickstart Guide: Physical AI & Humanoid Robotics Module 1

## Overview
This guide provides the essential setup instructions to get started with Module 1: The Robotic Nervous System. This module covers ROS 2 fundamentals and humanoid robotics using the "athena" humanoid model.

## Prerequisites
- Ubuntu 22.04 LTS
- Basic understanding of Python and machine learning concepts
- At least 8GB RAM (16GB recommended for simulation)
- Multi-core processor (Intel i5 or AMD equivalent minimum)

## Option 1: Docker Setup (Recommended)
For quickest setup with guaranteed compatibility:

```bash
# Clone the repository
git clone https://github.com/yourname/physical-ai-book.git
cd physical-ai-book

# Navigate to module 1
cd module1

# Build the Docker image (this may take 5-10 minutes)
docker build -t physical-ai-module1 .

# Run the container with GUI support (Linux)
xhost +local:docker
docker run -it --rm \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd):/workspace" \
  --device=/dev/dri:/dev/dri \
  --privileged \
  --name=physical-ai-dev \
  physical-ai-module1

# For Windows with WSL2, additional X-server setup required
```

## Option 2: Native Installation
For more experienced users who prefer native installation:

### 1. Install Ubuntu 22.04
If not already installed, set up Ubuntu 22.04 LTS.

### 2. Install ROS 2 Iron
```bash
# Set locale
sudo locale-gen en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Iron Desktop
sudo apt update
sudo apt install -y ros-iron-desktop
sudo apt install -y python3-rosdep2
sudo rosdep init
rosdep update

# Source ROS 2
echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install Additional Dependencies
```bash
# Install colcon for building packages
sudo apt install -y python3-colcon-common-extensions

# Install Gazebo Harmonic for simulation
sudo apt install -y gazebo libgazebo-dev

# Install Python dependencies
sudo apt install -y python3-pip python3-rosinstall python3-rosinstall-generator python3-wstool

# Install ROS 2 Gazebo packages
sudo apt install -y ros-iron-gazebo-* ros-iron-ros2-control* ros-iron-ros2-controllers*

# Install Xacro
sudo apt install -y ros-iron-xacro

# Install visualization tools
sudo apt install -y ros-iron-rviz2 ros-iron-robot-state-publisher ros-iron-joint-state-publisher-gui
```

### 4. Prepare Development Workspace
```bash
# Create workspace
mkdir -p ~/athena_ws/src
cd ~/athena_ws

# Clone the repository
git clone https://github.com/yourname/physical-ai-book.git src/physical-ai-book

# Create symbolic links to the module packages
ln -s src/physical-ai-book/module1/athena_description src/
ln -s src/physical-ai-book/module1/athena_bringup src/
ln -s src/physical-ai-book/module1/athena_control src/
ln -s src/physical-ai-book/module1/athena_gazebo src/
ln -s src/physical-ai-book/module1/athena_examples src/

# Source ROS 2 and build the workspace
source /opt/ros/iron/setup.bash
colcon build --packages-select athena_description athena_bringup athena_control athena_gazebo
source install/setup.bash
```

## Verifying Setup

### 1. Check ROS 2 Installation
```bash
source /opt/ros/iron/setup.bash
ros2 --version
```

### 2. Test Basic ROS 2 Functionality
```bash
# In one terminal
source ~/athena_ws/install/setup.bash
ros2 run demo_nodes_cpp talker

# In another terminal
source ~/athena_ws/install/setup.bash
ros2 run demo_nodes_py listener
```

### 3. Launch the "Athena" Humanoid in Simulation
```bash
# Source the workspace
source ~/athena_ws/install/setup.bash

# Launch Gazebo simulation with the athena robot
ros2 launch athena_gazebo athena_world.launch.py

# In another terminal, verify robot is loaded
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

### 4. Run a Basic Example
```bash
# Source the workspace
source ~/athena_ws/install/setup.bash

# Navigate to examples
cd src/physical-ai-book/module1/athena_examples

# Run the basic publisher example
python3 src/chapter2_publisher_subscriber.py
```

## Troubleshooting

### Common Issues:
1. **"command not found" for ROS commands**:
   - Ensure you've sourced the setup.bash file
   - Check that ROS 2 Iron is installed: `dpkg -l | grep ros-iron`

2. **Gazebo fails to start**:
   - Ensure you have a graphical environment
   - Check graphics drivers: `lspci | grep -i vga`

3. **Colcon build fails**:
   - Verify all dependencies are installed
   - Check if there are missing package dependencies

4. **"No space left" error during Docker build**:
   - Clear Docker cache: `docker system prune -a`
   - Free up disk space on your system

## Next Steps
- Proceed to Chapter 1: "From Digital AI to Embodied Intelligence"
- Follow along with the examples in the textbook
- Practice with the exercises at the end of each chapter
- Use the companion GitHub repository for reference implementations

## Performance Expectations
- Code examples should run with <100ms latency for AI-robot communication (requirement SC-013)
- Docker environment should set up in <5 minutes (requirement SC-010)
- Simulation should maintain >30 FPS on recommended hardware
- All exercises should be completable with 85% success rate (requirement SC-001)

For additional support, check the FAQ in the textbook or visit the companion GitHub repository issues page.