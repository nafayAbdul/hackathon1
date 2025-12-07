# Module 1: The Robotic Nervous System

This module covers the foundational aspects of ROS 2 and humanoid robotics as part of the Physical AI and Humanoid Robotics book. This module focuses on the ROS 2 "nervous system" that allows robots to function.

## Overview

Module 1: The Robotic Nervous System covers:
- From Digital AI to Embodied Intelligence
- ROS 2 Humble/Iron Deep Dive (Nodes, Topics, Services, Actions)
- rclpy – Bridging Python AI Agents to Robots
- URDF/Xacro Mastery for Humanoids
- Building Your First ROS 2 Humanoid Package

## Prerequisites

- Basic knowledge of Python and machine learning concepts
- Familiarity with command-line tools
- Access to a computer capable of running Ubuntu 22.04 (or equivalent virtualization)

## Setup

### Option 1: Docker Setup (Recommended)
The fastest way to get started is using the provided Docker setup:

1. **Install Docker** (if not already installed):
   ```bash
   # For Ubuntu:
   sudo apt update
   sudo apt install docker.io
   sudo usermod -aG docker $USER
   # Log out and back in for group changes to take effect
   ```

2. **Clone the companion repository**:
   ```bash
   git clone https://github.com/yourname/physical-ai-book.git
   cd physical-ai-book
   ```

3. **Build and run the Docker container**:
   ```bash
   cd module1
   docker build -t physical-ai-module1 .
   docker run -it --rm -v $(pwd):/workspace --name physical-ai-dev physical-ai-module1
   ```

4. **Inside the container, verify ROS 2 Iron is available**:
   ```bash
   source /opt/ros/iron/setup.bash
   ros2 --version
   ```

### Option 2: Native Ubuntu 22.04 Setup
If you prefer a native installation:

1. **Install Ubuntu 22.04** (or use an existing installation)

2. **Install ROS 2 Iron**:
   ```bash
   # Set locale
   sudo locale-gen en_US.UTF-8
   export LANG=en_US.UTF-8

   # Setup sources
   sudo apt update && sudo apt install -y curl gnupg lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
   
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install ROS 2 Iron packages
   sudo apt update
   sudo apt install -y ros-iron-desktop
   sudo apt install -y python3-rosdep2
   sudo rosdep init
   rosdep update
   ```

3. **Source ROS 2 in your shell**:
   ```bash
   echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

4. **Install additional dependencies**:
   ```bash
   sudo apt install -y python3-colcon-common-extensions
   sudo apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool
   sudo apt install -y gazebo libgazebo-dev
   sudo apt install -y ros-iron-gazebo-ros-pkgs ros-iron-gazebo-ros2-control
   sudo apt install -y ros-iron-xacro ros-iron-joint-state-publisher
   sudo apt install -y ros-iron-robot-state-publisher ros-iron-ros2-control
   sudo apt install -y ros-iron-ros2-controllers
   ```

## Setting up the "Athena" Humanoid Model

1. **Clone the repository** (if not done already):
   ```bash
   git clone https://github.com/yourname/physical-ai-book.git
   cd physical-ai-book/module1
   ```

2. **Create a ROS 2 workspace**:
   ```bash
   mkdir -p ~/athena_ws/src
   cd ~/athena_ws
   ```

3. **Copy the athena packages**:
   ```bash
   cp -r ~/physical-ai-book/module1/athena_description src/
   cp -r ~/physical-ai-book/module1/athena_bringup src/
   cp -r ~/physical-ai-book/module1/athena_control src/
   cp -r ~/physical-ai-book/module1/athena_gazebo src/
   ```

4. **Build the workspace**:
   ```bash
   cd ~/athena_ws
   source /opt/ros/iron/setup.bash
   colcon build --packages-select athena_description athena_bringup athena_control athena_gazebo
   source install/setup.bash
   ```

## Running Examples

After setting up your workspace, you can run example nodes from athena_examples:

```bash
cd ~/athena_ws
source install/setup.bash

# Run the chapter 2 basic publisher/subscriber example
ros2 run athena_examples chapter2_publisher_subscriber

# In another terminal, listen to the messages
ros2 topic echo /chatter std_msgs/msg/String
```

## Troubleshooting

1. **"command not found" for ROS 2 commands**:
   - Ensure you've sourced the setup.bash file: `source /opt/ros/iron/setup.bash`
   - Add the source command to your `.bashrc` file for persistence

2. **Gazebo doesn't start or shows errors**:
   - Make sure you have a GUI environment or X11 forwarding set up
   - Try running: `gazebo --verbose` for detailed error messages

3. **"Package not found" errors**:
   - Ensure you've built the workspace: `colcon build`
   - Check that you've sourced the workspace: `source install/setup.bash`

## Contents

- `athena_description/` - URDF models and mesh files for the "athena" humanoid
- `athena_bringup/` - Launch files to start the complete system
- `athena_control/` - Controllers for the humanoid robot
- `athena_gazebo/` - Gazebo simulation files for "athena"
- `athena_examples/` - Code examples from the book chapters
- `Dockerfile` - Container setup for quick environment
- `chapter[1-5]_*.md` - The five book chapters

## License

This project is licensed under the terms specified in the main repository.