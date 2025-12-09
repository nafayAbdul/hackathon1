# Data Model: Module 3 - AI-Robot Brain with Isaac Platform

## Overview
This document outlines the key data models and structures relevant to Module 3 of the Physical AI and Humanoid Robotics textbook, focusing on the NVIDIA Isaac Platform components.

## Key Entities

### 1. Athena Humanoid Model
The 23-DoF simplified Unitree G1/generic biped model used throughout the module.

**Fields and Properties:**
- **Joints (23)**: left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee, left_ankle_pitch, left_ankle_roll, right_hip_yaw, right_hip_roll, right_hip_pitch, right_knee, right_ankle_pitch, right_ankle_roll, torso_yaw, torso_roll, torso_pitch, left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow, left_wrist_pitch, left_wrist_yaw, right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow, right_wrist_pitch, right_wrist_yaw
- **Links**: pelvis, thighs, shins, feet, torso, upper arms, lower arms, hands
- **Physical Properties**: mass, inertia, center of mass
- **Articulation**: joint limits, drive properties, transmission parameters
- **Visual Properties**: materials, textures, geometry

**Validation Rules:**
- Joint angles must remain within specified limits
- Center of mass must be maintained within support polygon for balance

### 2. Isaac Sim Environment
Photorealistic simulation environment with RTX ray tracing, 1 kHz physics, and domain randomization capabilities.

**Fields and Properties:**
- **Physics Parameters**: gravity, friction, restitution, solver settings
- **Rendering Parameters**: RTX ray tracing settings, lighting, camera properties
- **Scene Elements**: environment geometry, objects, lighting conditions
- **Domain Randomization**: parameter ranges for randomization during training

**State Transitions:**
- Initialization → Running → Paused → Stopped

### 3. Isaac ROS 2 Components
Hardware-accelerated perception pipeline using NITROS and GEMs for SLAM, detection, and pose estimation.

**Fields and Properties:**
- **NITROS Transport**: optimized data formats, compression settings, bandwidth management
- **GEMs (GPU-Enhanced Modules)**: AprilTag detector, CuVSLAM, CuINS, foundation models
- **Sensor Processing**: camera, IMU, LiDAR data pipelines
- **Perception Outputs**: pose estimates, object detections, semantic segmentation

**Validation Rules:**
- Data streams must maintain consistent timing for proper sensor fusion
- Perceptual outputs must meet minimum confidence thresholds

### 4. Training Pipeline
Reinforcement learning workflow using Isaac Gym/Orbit/Lab to train policies for humanoid locomotion.

**Fields and Properties:**
- **Observation Space**: 47-dimensional state vector (positions, velocities, IMU data)
- **Action Space**: 23-dimensional action vector (joint commands)
- **Reward Function**: components for balance, forward progress, energy efficiency
- **Domain Randomization Parameters**: mass, friction, com offset, motor strength ranges
- **Training Metrics**: episode duration, success rate, convergence indicators

**State Transitions:**
- Environment Setup → Training Loop → Policy Evaluation → Model Export

### 5. Sim-to-Real Transfer Components
System identification, actuator modeling, and latency compensation for deployment on real hardware.

**Fields and Properties:**
- **System Identification Parameters**: mass properties, friction coefficients, actuator dynamics
- **Latency Compensation**: command buffering, predictive control, state estimation
- **Domain Randomization Schedules**: parameter ranges that evolve during training
- **Real-World Deployment**: ONNX model, control frequency, sensor integration

**Validation Rules:**
- Real-world performance must meet minimum stability criteria
- Policies must maintain balance during deployment

## Relationships

### Entity Relationships
- **Athena Humanoid Model** is used by **Isaac Sim Environment** for simulation
- **Isaac ROS 2 Components** process sensor data from **Isaac Sim Environment** (and real sensors)
- **Training Pipeline** operates on **Athena Humanoid Model** within **Isaac Sim Environment**
- **Sim-to-Real Transfer Components** translate policies from **Training Pipeline** to real-world deployment

### Data Flow
- Simulation → Perception → Planning → Control → Actuation
- Real-world sensors → Perception → Planning → Control → Humanoid (feedback loop)

## Schema Definitions

### Observation Vector Schema (for RL training)
```python
{
  "joint_positions": [float] * 23,      # Joint positions (radians)
  "joint_velocities": [float] * 23,     # Joint velocities (rad/s)
  "linear_acceleration": [float] * 3,   # IMU linear acceleration (m/s²)
  "angular_velocity": [float] * 3,      # IMU angular velocity (rad/s)
  "base_position": [float] * 3,         # Robot base position (m)
  "base_rotation": [float] * 4,         # Robot base rotation (quaternion)
  "commands": [float] * 2               # High-level commands (e.g., forward/back, turn)
}
```

### Action Vector Schema (for RL training)
```python
{
  "joint_targets": [float] * 23         # Target joint positions (radians)
}
```

### Domain Randomization Parameters Schema
```python
{
  "mass_range": [float, float],         # Multiplier range for link masses
  "friction_range": [float, float],     # Range for friction coefficients
  "com_offset_range": [float, float],   # Range for center of mass offsets
  "motor_strength_range": [float, float]# Multiplier range for motor strengths
}
```

## State Models

### Training State Model
```python
{
  "episode": int,                       # Current episode number
  "step": int,                          # Current step in episode
  "reward": float,                      # Current reward
  "done": bool,                         # Episode termination flag
  "obs": [float],                       # Current observation vector
  "action": [float],                    # Last action taken
  "episode_length": int,                # Steps in current episode
  "episode_reward": float,              # Cumulative reward in episode
  "success_rate": float                 # Success rate over recent episodes
}
```

## Validation Rules

### General Validation
- All angle values must be in radians
- All position values must be in meters
- All velocity values must be in appropriate units (rad/s for joints, m/s for linear)
- All force/torque values must be in appropriate units (N for forces, Nm for torques)

### Physics Validation
- Joint positions must remain within joint limits
- Center of mass must remain within support polygon for stable standing/walking
- Total torque applied to joints must not exceed actuator limits

### Performance Validation
- Simulation must maintain 60 FPS for interactive use
- RL training episodes must complete within reasonable time limits
- Real-world deployment must achieve minimum control frequency (200Hz+)