"""
Configuration Management for Different Hardware Tiers

This module provides configuration management for different hardware tiers
(Tier 0-4) as specified in the Module 4 requirements.
"""
import json
import yaml
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, asdict
import os
from pathlib import Path


@dataclass
class HardwareTierConfig:
    """
    Configuration for a specific hardware tier
    """
    tier: int
    name: str
    description: str
    min_gpu_vram_gb: float
    recommended_gpu_vram_gb: float
    min_cpu_cores: int
    min_ram_gb: int
    required_libraries: List[str]
    docker_image: str
    performance_requirements: Dict[str, Any]  # e.g., {"max_latency_ms": 220, "min_success_rate": 0.7}
    safety_settings: Dict[str, Any]
    features_enabled: List[str]


@dataclass
class VLAConfig:
    """
    VLA-specific configuration
    """
    model_name: str
    model_path: str
    precision: str  # e.g., "fp16", "fp32", "int8", "int4"
    image_size: List[int]  # [height, width]
    action_space_dim: int
    max_sequence_length: int
    inference_batch_size: int


@dataclass
class SystemConfig:
    """
    Overall system configuration
    """
    project_name: str
    module_version: str
    hardware_tier: int
    vla_config: VLAConfig
    tier_configs: List[HardwareTierConfig]
    model_paths: Dict[str, str]  # Mapping of model names to paths
    log_level: str
    enable_monitoring: bool


class ConfigManager:
    """
    Manager for configuration across different hardware tiers
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize the configuration manager
        
        Args:
            config_path: Path to a configuration file (optional)
        """
        self.config_path = config_path
        self.system_config = None
        self.default_configs = self._get_default_configs()
        
        if config_path and os.path.exists(config_path):
            self.load_config(config_path)
        else:
            # Use default configuration for Tier 2 (Jetson Orin NX 16GB) as default
            self.system_config = self._create_default_system_config(2)
    
    def _get_default_configs(self) -> Dict[int, HardwareTierConfig]:
        """
        Get default configurations for hardware tiers 0-4
        
        Returns:
            Dictionary mapping tier numbers to configurations
        """
        return {
            0: HardwareTierConfig(
                tier=0,
                name="Tier 0 (Cloud)",
                description="Cloud-based development requiring 8+ GB VRAM",
                min_gpu_vram_gb=8.0,
                recommended_gpu_vram_gb=24.0,
                min_cpu_cores=8,
                min_ram_gb=32,
                required_libraries=["torch", "transformers", "openvla", "whisper", "accelerate"],
                docker_image="nvcr.io/nvidia/cuda:12.6-devel-ubuntu22.04",
                performance_requirements={"max_latency_ms": 90, "min_success_rate": 0.95},
                safety_settings={"enable_simulation_safety": True, "max_velocity_scale": 1.0},
                features_enabled=["full_model", "high_quality_vision", "advanced_language_processing", "real_time_inference"]
            ),
            1: HardwareTierConfig(
                tier=1,
                name="Tier 1 (Simulation)",
                description="Isaac Sim with realistic physics and rendering",
                min_gpu_vram_gb=12.0,
                recommended_gpu_vram_gb=24.0,
                min_cpu_cores=8,
                min_ram_gb=32,
                required_libraries=["torch", "transformers", "openvla", "whisper", "omniisaacgymenvs", "isaacsim"],
                docker_image="nvcr.io/nvidia/isaac-sim:4.2.0",
                performance_requirements={"max_latency_ms": 120, "min_success_rate": 0.90},
                safety_settings={"enable_simulation_safety": True, "max_velocity_scale": 0.8},
                features_enabled=["simulation", "physics", "rendering", "domain_randomization"]
            ),
            2: HardwareTierConfig(
                tier=2,
                name="Tier 2 (Edge GPU - Jetson Orin NX 16GB)",
                description="Deployment on Jetson Orin NX with 16GB VRAM",
                min_gpu_vram_gb=12.0,
                recommended_gpu_vram_gb=16.0,
                min_cpu_cores=8,
                min_ram_gb=16,
                required_libraries=["torch", "transformers", "openvla", "whisper", "tensorrt", "jetson-utils"],
                docker_image="nvcr.io/nvidia/jetson-ml:r35.4.1",
                performance_requirements={"max_latency_ms": 220, "min_success_rate": 0.85},
                safety_settings={"enable_hardware_safety": True, "max_velocity_scale": 0.6},
                features_enabled=["quantized_model", "optimized_inference", "power_efficiency"]
            ),
            3: HardwareTierConfig(
                tier=3,
                name="Tier 3 (NVIDIA Isaac Platform)",
                description="NVIDIA Isaac Platform with real sensors and simulated actuators",
                min_gpu_vram_gb=16.0,
                recommended_gpu_vram_gb=24.0,
                min_cpu_cores=16,
                min_ram_gb=32,
                required_libraries=["torch", "transformers", "openvla", "whisper", "isaac_ros", "ros2"],
                docker_image="nvcr.io/nvidia/isaac-ros:ros2-humble-isaac-ros-2.2.0",
                performance_requirements={"max_latency_ms": 300, "min_success_rate": 0.80},
                safety_settings={"enable_robot_safety": True, "max_velocity_scale": 0.5},
                features_enabled=["ros_integration", "sensor_processing", "actuator_control", "perception_pipeline"]
            ),
            4: HardwareTierConfig(
                tier=4,
                name="Tier 4 (Real Humanoid Hardware)",
                description="Real humanoid hardware with safety protocols",
                min_gpu_vram_gb=16.0,
                recommended_gpu_vram_gb=24.0,
                min_cpu_cores=16,
                min_ram_gb=32,
                required_libraries=["torch", "transformers", "openvla", "whisper", "ros2", "control_msgs"],
                docker_image="nvcr.io/nvidia/isaac-ros:ros2-humble-isaac-ros-2.2.0",
                performance_requirements={"max_latency_ms": 400, "min_success_rate": 0.70},
                safety_settings={"enable_physical_safety": True, "max_velocity_scale": 0.3, "emergency_stop_enabled": True},
                features_enabled=["real_hardware_control", "safety_system", "collision_avoidance", "torque_control"]
            )
        }
    
    def _create_default_system_config(self, tier: int) -> SystemConfig:
        """
        Create a default system configuration for a specific tier
        
        Args:
            tier: Hardware tier (0-4)
            
        Returns:
            SystemConfig object
        """
        tier_config = self.default_configs[tier]
        
        vla_config = VLAConfig(
            model_name="openvla/openvla-7b",
            model_path=f"./models/openvla-7b-tier{tier}",
            precision="fp16" if tier > 1 else "fp32",  # Use lower precision on less powerful hardware
            image_size=[224, 224],
            action_space_dim=7,
            max_sequence_length=512,
            inference_batch_size=1 if tier > 2 else 4  # Smaller batch size for resource-constrained systems
        )
        
        return SystemConfig(
            project_name="Module4-VLA-System",
            module_version="1.0.0",
            hardware_tier=tier,
            vla_config=vla_config,
            tier_configs=list(self.default_configs.values()),
            model_paths={
                "vla_model": f"./models/openvla-7b-tier{tier}",
                "speech_model": "./models/whisper-large-v3",
                "language_model": "./models/llama-3.1-8b-instruct"
            },
            log_level="INFO",
            enable_monitoring=True
        )
    
    def load_config(self, config_path: str):
        """
        Load configuration from a file
        
        Args:
            config_path: Path to the configuration file
        """
        with open(config_path, 'r') as f:
            if config_path.endswith('.json'):
                config_data = json.load(f)
            elif config_path.endswith(('.yaml', '.yml')):
                config_data = yaml.safe_load(f)
            else:
                raise ValueError(f"Unsupported config file format: {config_path}")
        
        # Parse the configuration data into appropriate objects
        self.system_config = self._parse_config_data(config_data)
    
    def _parse_config_data(self, config_data: Dict[str, Any]) -> SystemConfig:
        """
        Parse configuration data into SystemConfig object
        
        Args:
            config_data: Raw configuration data from file
            
        Returns:
            SystemConfig object
        """
        vla_config = VLAConfig(**config_data.get('vla_config', {}))
        
        tier_configs = []
        for tier_data in config_data.get('tier_configs', []):
            tier_configs.append(HardwareTierConfig(**tier_data))
        
        return SystemConfig(
            project_name=config_data.get('project_name', 'Module4-VLA-System'),
            module_version=config_data.get('module_version', '1.0.0'),
            hardware_tier=config_data.get('hardware_tier', 2),
            vla_config=vla_config,
            tier_configs=tier_configs,
            model_paths=config_data.get('model_paths', {}),
            log_level=config_data.get('log_level', 'INFO'),
            enable_monitoring=config_data.get('enable_monitoring', True)
        )
    
    def save_config(self, config_path: str):
        """
        Save current configuration to a file
        
        Args:
            config_path: Path to save the configuration file
        """
        if not self.system_config:
            raise ValueError("No configuration to save")
        
        config_dict = {
            "project_name": self.system_config.project_name,
            "module_version": self.system_config.module_version,
            "hardware_tier": self.system_config.hardware_tier,
            "vla_config": asdict(self.system_config.vla_config),
            "tier_configs": [asdict(config) for config in self.system_config.tier_configs],
            "model_paths": self.system_config.model_paths,
            "log_level": self.system_config.log_level,
            "enable_monitoring": self.system_config.enable_monitoring
        }
        
        with open(config_path, 'w') as f:
            if config_path.endswith('.json'):
                json.dump(config_dict, f, indent=2)
            elif config_path.endswith(('.yaml', '.yml')):
                yaml.dump(config_dict, f, default_flow_style=False)
            else:
                raise ValueError(f"Unsupported config file format: {config_path}")
    
    def get_tier_config(self, tier: int) -> HardwareTierConfig:
        """
        Get the configuration for a specific hardware tier
        
        Args:
            tier: Hardware tier (0-4)
            
        Returns:
            HardwareTierConfig object
        """
        if tier in self.default_configs:
            return self.default_configs[tier]
        else:
            raise ValueError(f"Unsupported hardware tier: {tier}")
    
    def set_hardware_tier(self, tier: int):
        """
        Set the current hardware tier and update configuration accordingly
        
        Args:
            tier: New hardware tier (0-4)
        """
        if tier not in self.default_configs:
            raise ValueError(f"Unsupported hardware tier: {tier}")
        
        # Update the system configuration for the new tier
        self.system_config = self._create_default_system_config(tier)
    
    def get_current_tier_config(self) -> HardwareTierConfig:
        """
        Get the configuration for the currently set hardware tier
        
        Returns:
            HardwareTierConfig object
        """
        if not self.system_config:
            raise ValueError("Configuration not initialized")
        
        return self.get_tier_config(self.system_config.hardware_tier)
    
    def validate_for_tier(self, tier: Optional[int] = None) -> List[str]:
        """
        Validate if the system can run on the specified tier
        
        Args:
            tier: Hardware tier to validate against (defaults to current tier)
            
        Returns:
            List of validation errors (empty if valid)
        """
        if tier is None:
            if not self.system_config:
                raise ValueError("Configuration not initialized")
            tier = self.system_config.hardware_tier
        
        tier_config = self.get_tier_config(tier)
        errors = []
        
        # Validation would include checking hardware resources,
        # available libraries, etc. This is a simplified version.
        if not os.path.exists(tier_config.docker_image.replace(":", "_")):
            errors.append(f"Docker image {tier_config.docker_image} may not be available")
        
        # Check required libraries (simplified check)
        for lib in tier_config.required_libraries:
            try:
                __import__(lib.split(".")[0])  # Import the main module
            except ImportError:
                errors.append(f"Required library {lib} is not installed")
        
        # Performance requirements check could be added here
        
        return errors
    
    def get_model_path_for_tier(self, model_type: str, tier: Optional[int] = None) -> str:
        """
        Get the appropriate model path for a specific model type and tier
        
        Args:
            model_type: Type of model (e.g., 'vla', 'speech', 'language')
            tier: Hardware tier (defaults to current tier)
            
        Returns:
            Path to the appropriate model
        """
        if tier is None:
            if not self.system_config:
                raise ValueError("Configuration not initialized")
            tier = self.system_config.hardware_tier
        
        # Determine the appropriate model based on tier requirements
        if model_type == "vla":
            # Use quantized models for tiers 2+
            if tier >= 2:
                return f"./models/openvla-7b-quantized-tier{tier}"
            else:
                return f"./models/openvla-7b-tier{tier}"
        elif model_type == "speech":
            # Use smaller models for resource-constrained systems
            if tier >= 3:
                return "./models/whisper-tiny"
            else:
                return "./models/whisper-large-v3"
        elif model_type == "language":
            # Adjust based on available resources
            if tier >= 3:
                return "./models/phi-3-mini-4k-instruct"
            else:
                return "./models/llama-3.1-8b-instruct"
        else:
            raise ValueError(f"Unknown model type: {model_type}")
    
    def adjust_config_for_performance(self, target_latency: float):
        """
        Adjust configuration to meet a target latency requirement
        
        Args:
            target_latency: Target latency in milliseconds
        """
        if not self.system_config:
            raise ValueError("Configuration not initialized")
        
        # Adjust batch size, precision, etc. based on target latency
        if target_latency < 100:
            # High performance mode, keep settings as is
            pass
        elif target_latency < 200:
            # Medium performance mode
            self.system_config.vla_config.inference_batch_size = 2
            self.system_config.vla_config.precision = "fp16"
        else:
            # Low performance mode, reduce demands
            self.system_config.vla_config.inference_batch_size = 1
            self.system_config.vla_config.precision = "int8"


def load_system_config(config_path: Optional[str] = None) -> ConfigManager:
    """
    Convenience function to load system configuration
    
    Args:
        config_path: Path to the configuration file (optional)
        
    Returns:
        ConfigManager instance
    """
    return ConfigManager(config_path)


# Example configuration files that would be created
EXAMPLE_CONFIG = {
    "project_name": "Module4-VLA-System",
    "module_version": "1.0.0",
    "hardware_tier": 2,
    "vla_config": {
        "model_name": "openvla/openvla-7b",
        "model_path": "./models/openvla-7b-tier2",
        "precision": "fp16",
        "image_size": [224, 224],
        "action_space_dim": 7,
        "max_sequence_length": 512,
        "inference_batch_size": 1
    },
    "tier_configs": [
        {
            "tier": 2,
            "name": "Tier 2 (Edge GPU - Jetson Orin NX 16GB)",
            "description": "Deployment on Jetson Orin NX with 16GB VRAM",
            "min_gpu_vram_gb": 12.0,
            "recommended_gpu_vram_gb": 16.0,
            "min_cpu_cores": 8,
            "min_ram_gb": 16,
            "required_libraries": ["torch", "transformers", "openvla", "whisper", "tensorrt", "jetson-utils"],
            "docker_image": "nvcr.io/nvidia/jetson-ml:r35.4.1",
            "performance_requirements": {"max_latency_ms": 220, "min_success_rate": 0.85},
            "safety_settings": {"enable_hardware_safety": True, "max_velocity_scale": 0.6},
            "features_enabled": ["quantized_model", "optimized_inference", "power_efficiency"]
        }
    ],
    "model_paths": {
        "vla_model": "./models/openvla-7b-tier2",
        "speech_model": "./models/whisper-large-v3",
        "language_model": "./models/llama-3.1-8b-instruct"
    },
    "log_level": "INFO",
    "enable_monitoring": True
}