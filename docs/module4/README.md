# Module 4: Vision-Language-Action Models – From Voice to Physical Action (Weeks 11–13)

This directory contains the complete documentation for Module 4 of the Physical AI and Humanoid Robotics textbook.

## Overview

Module 4 focuses on Vision-Language-Action (VLA) models and creating complete voice-to-action pipelines for the "athena" humanoid robot using NVIDIA's Isaac Platform. The module covers:

1. **Chapter 16**: The VLA Revolution – OpenVLA, RT-2-X, Octo, π0, and the 2025 Landscape
2. **Chapter 17**: Building and Fine-tuning Your Own Vision-Language-Action Model
3. **Chapter 18**: The Complete Voice → Action Pipeline (Whisper → VLA → ROS 2)
4. **Chapter 19**: Multi-Modal Foundations – Adding Gesture, Gaze, and Long-Horizon Reasoning
5. **Chapter 20**: Sim-to-Real Transfer Cookbook – Making Athena Walk on Real Hardware

## Prerequisites

Before studying this module, you should have:

- Completed Modules 1-3 with a working "athena" humanoid robot
- An RTX 4070 Ti+ workstation with 32GB+ RAM
- Isaac Sim 2025.2.1, Isaac ROS 2.2.0, and Isaac Lab 1.3 installed
- Understanding of ROS 2 Iron and CUDA 12.6

## Technical Requirements

All examples are tested with:
- Ubuntu 22.04 LTS
- ROS 2 Iron (December 2025 version)
- Isaac Sim 2025.2.1
- Isaac ROS 2.2.0
- Isaac Lab 1.3
- RTX 4090 GPU for optimal performance

## Companion Assets

Code examples, USD assets, and training scripts are available at:
`github.com/yourname/physical-ai-book/tree/main/module4`

## The Legendary `isaacsim.run`

This module culminates with the legendary one-liner command that launches the complete autonomous humanoid system in a photorealistic apartment environment.