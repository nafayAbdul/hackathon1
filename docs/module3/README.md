# Module 3: The AI-Robot Brain – NVIDIA Isaac Platform

Welcome to Module 3 of the Physical AI and Humanoid Robotics textbook. This module focuses on creating advanced AI-robot brains using NVIDIA's Isaac Platform. You'll learn to master Isaac Sim 2025.2, implement hardware-accelerated perception with Isaac ROS 2, integrate navigation and manipulation systems, train reinforcement learning policies with Isaac Lab, and execute sim-to-real transfer.

## Module Structure

This module contains five comprehensive chapters:

- **Chapter 11**: Isaac Sim 2025 – From Installation to Photorealistic Humanoid Simulation
- **Chapter 12**: Isaac ROS 2 – Hardware-Accelerated Perception with NITROS and GEMs
- **Chapter 13**: Advanced Navigation & Manipulation for Bipedal Humanoids (Nav2 + MoveIt 2 + Isaac)
- **Chapter 14**: Reinforcement Learning at Scale with Isaac Gym, Isaac Orbit, and Isaac Lab
- **Chapter 15**: Sim-to-Real Transfer Cookbook – Making Athena Walk on Real Hardware

## Learning Objectives

By the end of this module, you will be able to:

1. Install and configure Isaac Sim 2025.2 with optimal performance for humanoid simulation
2. Implement hardware-accelerated perception using Isaac ROS 2 with NITROS and GEMs
3. Integrate Nav2 and MoveIt 2 for complete navigation and manipulation in Isaac Sim
4. Train complex humanoid policies using Isaac Lab 1.3 and rsl-rl
5. Execute successful sim-to-real transfer with domain randomization and latency compensation

## Prerequisites

Before starting this module, you should have:

- Completed Modules 1 and 2 with a working ROS 2 Iron + URDF humanoid named "athena"
- An RTX 4070 Ti+ workstation (or access to one)
- Understanding of Gazebo simulation concepts
- Experience with Python and ROS 2 development

## Technical Requirements

All examples in this module are tested with:

- Ubuntu 22.04 LTS
- ROS 2 Iron (December 2025 version)
- Isaac Sim 2025.2.1
- Isaac ROS 2.2.0
- Isaac Lab 1.3
- CUDA 12.6
- Python 3.10+

## Companion Assets

All USD assets, extensions, and training scripts for this module are available at:
`github.com/yourname/physical-ai-book/tree/main/module3`

## The Legendary "isaacsim.run"

The module culminates with the implementation of the legendary one-liner command `isaacsim.run` that launches the full autonomous humanoid in a photorealistic apartment environment.