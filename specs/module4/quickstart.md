# Quickstart Guide: Module 4 - Vision-Language-Action Models

## Overview
This quickstart guide provides a fast path to get started with Module 4: Vision-Language-Action Models – From Voice to Physical Action. It covers the essential steps to set up the environment, run basic examples, and understand the core concepts.

## Prerequisites
- Ubuntu 22.04 LTS
- Python 3.10
- ROS 2 Iron
- NVIDIA GPU with CUDA 12.6 support (minimum RTX 3060, recommended RTX 4080 or better)
- 32GB+ RAM (64GB recommended)
- 50GB+ free disk space

## Setup

### 1. Clone the Repository
```bash
git clone [repository-url]
cd pysical_ai
```

### 2. Install System Dependencies
```bash
# Install CUDA and NVIDIA drivers if not already installed
# Install dependencies
sudo apt update
sudo apt install python3-dev python3-pip git curl ffmpeg
```

### 3. Set up Python Environment
```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install --upgrade pip
```

### 4. Install Module 4 Dependencies
```bash
cd module4
pip install -r requirements.txt
```

### 5. Download Models
```bash
# This will download the required models to module4/models/
python scripts/download_models.py
# or manually:
# OpenVLA-7B: https://huggingface.co/openvla/openvla-7b
# Whisper Large v3: https://huggingface.co/openai/whisper-large-v3
# Llama 3.1 8B: https://huggingface.co/meta-llama/Llama-3.1-8B-Instruct
```

## Basic Usage Examples

### Example 1: Running OpenVLA Inference
```python
from module4.utils.vla_interface import initialize_vla_interface
from PIL import Image

# Initialize VLA interface
vla_interface = initialize_vla_interface()

# Load an image
image = Image.open("path/to/your/image.jpg")

# Run inference with an instruction
result = vla_interface.predict_action(image, "Pick up the red cube")

# Check the result
if result['success']:
    print(f"Robot action: {result['robot_action']}")
else:
    print(f"Error: {result['message']}")
```

### Example 2: Processing Voice Commands
```python
from module4.utils.speech_processing import initialize_speech_processor

# Initialize speech processor
speech_processor = initialize_speech_processor()

# Process an audio file
result = speech_processor.process_voice_command("path/to/audio.wav")

if result['success']:
    print(f"Recognized command: {result['command']['raw_command']}")
    
    # You can then pass this command to the VLA system
    # vla_result = vla_interface.predict_action(image, result['command']['raw_command'])
```

### Example 3: Executing Actions on Robot (Simulation)
```python
from module4.utils.hardware_abstraction import initialize_hardware_abstraction

# Initialize hardware abstraction layer (simulated)
hal = initialize_hardware_abstraction("athena", use_real_hardware=False)

# Connect to robot (simulated)
hal.connect()

# Execute an action
test_action = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
success = hal.execute_action(test_action, "joint_positions")

if success:
    print("Action executed successfully")
    print(f"Current joint states: {hal.get_joint_states()}")
else:
    print("Action execution failed")

# Disconnect
hal.disconnect()
```

## Hardware Tier Configuration

The VLA system supports multiple hardware tiers, each with different capabilities:

```python
from module4.utils.config import set_hardware_tier, get_current_vla_config

# Set to Tier 1 (RTX 4090 - highest performance)
set_hardware_tier(1)
config = get_current_vla_config()
print(f"Current config: {config}")

# Set to Tier 2 (Jetson Orin - edge deployment)
set_hardware_tier(2)
config = get_current_vla_config()
print(f"Edge config: {config}")
```

## Running Tests

To verify your setup is working correctly:

```bash
cd module4/tests
python test_infrastructure.py
```

Or run with pytest:
```bash
python -m pytest tests/ -v
```

## Safety System

The safety system is critical for real-world deployment:

```python
from module4.utils.safety_system import initialize_safety_system

# Initialize safety system with emergency stop callback
def emergency_stop():
    print("Emergency stop activated!")
    # Implement actual emergency stop logic
    return True

safety_system = initialize_safety_system("athena", emergency_stop)

# Validate an action before executing
test_action = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
validation = safety_system.validate_action(test_action, "joint_positions")

if validation['is_safe']:
    # Execute the action (in a real system)
    print("Action is safe to execute")
else:
    print(f"Action blocked: {validation['violations']}")
```

## Complete Example: Voice Command to Robot Action

Here's a complete example that ties everything together:

```python
from module4.utils.vla_interface import initialize_vla_interface
from module4.utils.speech_processing import initialize_speech_processor
from module4.utils.hardware_abstraction import initialize_hardware_abstraction
from module4.utils.safety_system import initialize_safety_system
from module4.utils.config import set_hardware_tier
from PIL import Image

def voice_to_action_demo():
    # Set hardware tier (use Tier 1 for development)
    set_hardware_tier(1)
    
    # Initialize components
    vla_interface = initialize_vla_interface()
    speech_processor = initialize_speech_processor()
    hal = initialize_hardware_abstraction("athena", use_real_hardware=False)
    
    # For this example, we'll simulate the speech input
    command_text = "Move to the red object"
    
    # In a real system, you'd get this from speech recognition:
    # audio_result = speech_processor.process_voice_command("path/to/audio.wav")
    # command_text = audio_result['command']['raw_command']
    
    # Get a current image (in real system, from robot's camera)
    # For demo, create a dummy image
    dummy_image = Image.new('RGB', (224, 224), color='red')
    
    # Get action from VLA model
    vla_result = vla_interface.predict_action(dummy_image, command_text)
    
    if vla_result['success']:
        print(f"VLA generated action: {vla_result['robot_action']}")
        
        # In a real system, connect to robot and execute
        # hal.connect()
        # success = hal.execute_action(vla_result['robot_action'], "joint_positions")
        # hal.disconnect()
        
        print("Demo completed successfully!")
    else:
        print(f"VLA prediction failed: {vla_result['message']}")

if __name__ == "__main__":
    voice_to_action_demo()
```

## Docker Setup (Alternative)

If you prefer using Docker:

```bash
# Build the Docker image
cd module4/docker
docker build -f Dockerfile.module4 -t vla-module4 .

# Run the container with GPU support
docker run --gpus all --shm-size=16gb -it vla-module4

# Inside the container, you can run the examples
```

## Troubleshooting

### Common Issues:
1. **CUDA/GPU Issues**: Ensure CUDA 12.6 and compatible GPU drivers are installed
2. **Memory Issues**: VLA models require significant VRAM. Use smaller models or reduce batch sizes
3. **Model Download**: Ensure you have sufficient bandwidth and disk space for model downloads

### Performance Tips:
1. Use model quantization for edge deployments
2. Adjust batch sizes based on available VRAM
3. Use appropriate hardware tier settings for your system
4. Consider using SSD storage for faster model loading

## Next Steps
- Read the full chapters in `docs/module4/` for detailed explanations
- Try the exercises in the exercise files
- Experiment with different hardware tier configurations
- Implement the complete Athena system in Chapter 20