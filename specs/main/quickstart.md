# Quickstart Guide: Module 3 - AI-Robot Brain with Isaac Platform

## Overview
This quickstart guide provides a step-by-step introduction to the key concepts and tools covered in Module 3 of the Physical AI and Humanoid Robotics textbook. This module focuses on the NVIDIA Isaac Platform for creating advanced AI-robot brain systems.

## Prerequisites
Before starting this module, ensure you have:

1. **Hardware Requirements**:
   - RTX 4070 Ti+ GPU (or equivalent)
   - 32GB RAM minimum
   - Ubuntu 22.04 LTS

2. **Software Requirements**:
   - ROS 2 Iron (December 2025 version)
   - CUDA 12.6
   - Isaac Sim 2025.2.1
   - Isaac ROS 2.2.0
   - Isaac Lab 1.3

3. **Completion of Modules 1-2**:
   - Working "athena" 23-DoF humanoid robot in Gazebo

## Installation and Setup

### 1. Install Isaac Sim 2025.2.1
```bash
# Create a dedicated conda environment
conda create -n isaacsim python=3.10
conda activate isaacsim

# Install Isaac Sim wheel (requires NVIDIA Developer Account)
pip install --extra-index-url https://pypi.ngc.nvidia.com --index-url https://pypi.ngc.nvidia.com --trusted-host pypi.ngc.nvidia.com --user isaacsim

# Verify installation
python -m omni.isaac.sim.python.gym --no-window --num_envs 1
```

### 2. Install Isaac ROS 2.2.0
```bash
# Pull the Isaac ROS 2 container with all GEMs
docker pull nvcr.io/nvidia/isaac-ros:ros2-humble-isaac-ros-2.2.0

# Run the container with GPU access
docker run --gpus all -it --rm \
  --network host \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --volume /dev:/dev \
  --volume /tmp:/tmp \
  nvcr.io/nvidia/isaac-ros:ros2-humble-isaac-ros-2.2.0
```

### 3. Install Isaac Lab 1.3
```bash
# Isaac Lab installation (detailed steps will be covered in Chapter 14)
# For now, ensure prerequisites are met:
pip install torch==2.3.0
pip install rsl-rl==1.0.2
```

## Chapter 11 Quickstart: Isaac Sim Installation and Setup

### Objective
Install Isaac Sim and load your "athena" humanoid model in USD format.

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage

# Initialize the world
world = World(stage_units_in_meters=1.0)

# Add your athena robot to the stage
add_reference_to_stage(
    usd_path="/path/to/athena/athena.usd",  # Your converted USD
    prim_path="/World/Robot"
)

# Reset the world to apply the robot
world.reset()

# Configure physics for humanoid simulation
world.physics_scene.set_gravity(-9.81)
world.set_physics_dt(1.0/1000.0)  # 1 kHz physics
```

### Key Concepts:
- USD (Universal Scene Description) format
- 1 kHz physics engine
- RTX ray-traced rendering

## Chapter 12 Quickstart: Isaac ROS 2 Perception

### Objective
Set up hardware-accelerated perception using Isaac ROS 2.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class IsaacROSPipeline(Node):
    def __init__(self):
        super().__init__('isaac_ros_pipeline')
        
        # Subscribe to camera with NITROS optimization
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
    
    def image_callback(self, msg):
        # Process image using hardware acceleration
        # Implementation details in Chapter 12
        pass

def main(args=None):
    rclpy.init(args=args)
    node = IsaacROSPipeline()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Key Concepts:
- NITROS (NVIDIA Isaac Transport for Robotics)
- GEMs (GPU-Enhanced Modules)
- CuVSLAM, AprilTag, CuINS

## Chapter 13 Quickstart: Navigation and Manipulation

### Objective
Integrate Nav2 and MoveIt 2 for bipedal navigation and manipulation.

```yaml
# Example Nav2 configuration for bipedal robots (nav2_params.yaml)
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5  # Reduced for balance
      max_vel_y: 0.1
      max_vel_theta: 0.3
```

### Key Concepts:
- SMAC planner for legged robots
- Bipedal navigation constraints
- Perception-to-manipulation pipeline

## Chapter 14 Quickstart: Reinforcement Learning

### Objective
Train a basic walking policy using Isaac Lab.

```python
import torch
from omni.isaac.orbit_tasks.base.vec_env import VecEnv

class AthenaRLTask:
    def __init__(self, num_envs, device):
        self.num_envs = num_envs
        self.device = device
        self.num_actions = 23  # For 23 DoF athena humanoid
        self.num_observations = 47  # Position, velocity, IMU data, etc.

    def reset(self):
        # Reset all environments
        pass

    def step(self, actions):
        # Apply actions and step simulation
        pass
```

### Key Concepts:
- Isaac Lab 1.3 for RL
- rsl-rl for humanoid locomotion
- Domain randomization

## Chapter 15 Quickstart: Sim-to-Real Transfer

### Objective
Execute zero-shot transfer of trained policy to real hardware.

```python
import onnxruntime as ort
import numpy as np

class ZeroShotTransfer:
    def __init__(self, onnx_model_path):
        # Load the trained ONNX policy
        self.session = ort.InferenceSession(onnx_model_path)
    
    def compute_action(self, observation):
        # Get action from the trained policy
        obs_input = observation.reshape(1, -1).astype(np.float32)
        input_name = self.session.get_inputs()[0].name
        output = self.session.run(None, {input_name: obs_input})
        action = output[0][0]  # Remove batch dimension
        return action
```

### Key Concepts:
- System identification
- Latency compensation
- ONNX export for deployment

## The Legendary "isaacsim.run" Command

After completing all chapters, you'll have the legendary one-liner command that launches the complete autonomous humanoid:

```bash
# The legendary command that launched the full autonomous humanoid
./isaacsim.run
```

This command orchestrates:
- Isaac Sim with photorealistic "athena" humanoid
- Isaac ROS 2 perception stack
- Trained walking policy
- Autonomous operation in apartment environment

## Next Steps

After completing this quickstart:
1. Proceed to Chapter 11 for detailed Isaac Sim installation and setup
2. Work through each chapter sequentially, building on previous knowledge
3. Complete the hands-on exercises at the end of each chapter
4. Aim for the final challenge: making "athena" walk 5 meters using only the policy trained in simulation