# Module 3: The AI-Robot Brain – NVIDIA Isaac Platform

This directory contains all the code, configuration files, and examples for Module 3 of the Physical AI and Humanoid Robotics textbook.

## Contents

- `isaacsim.run` - The legendary one-liner script to launch the complete autonomous humanoid system
- `requirements.txt` - Python dependencies for the Isaac platform
- `Dockerfile` - Docker configuration for Isaac ROS 2 environment
- `athena_config.yaml` - Configuration file for the Athena humanoid robot in Isaac Sim
- Documentation files in `../docs/module3/` - Complete module documentation with all chapters

## Getting Started

1. Install Isaac Sim 2025.2 following Chapter 11 instructions
2. Set up Isaac ROS 2 with the configuration in this directory
3. Build the Docker environment: `docker build -f Dockerfile -t athena-isaac-ros .`
4. Run the legendary command: `./isaacsim.run`

## Prerequisites

- NVIDIA GPU with RTX capabilities
- Ubuntu 22.04 with ROS 2 Iron
- Isaac Sim 2025.2.1
- Isaac ROS 2.2.0
- Isaac Lab 1.3

## Technical Requirements

All examples in this module are tested with:
- RTX 4070 Ti or better
- 32GB RAM minimum
- CUDA 12.6
- Python 3.10+

## Companion Assets

For the complete USD models, training scripts, and additional assets referenced in the module, see: 
`github.com/yourname/physical-ai-book/tree/main/module3`