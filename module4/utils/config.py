"""
Configuration management for Module 4
Handles configuration for different hardware tiers (0-4)
"""
import json
import yaml
import os
from typing import Dict, Any, Optional, Union
from dataclasses import dataclass, asdict
from pathlib import Path
import logging

logger = logging.getLogger(__name__)

@dataclass
class HardwareTierConfig:
    """
    Configuration for a specific hardware tier
    """
    # Tier identifier
    tier: int
    
    # Hardware specifications
    gpu_model: str
    gpu_vram_gb: int
    cpu_cores: int
    system_ram_gb: int
    storage_type: str  # "SSD", "HDD", "NVMe"
    
    # Performance parameters
    max_inference_latency_ms: float
    max_vla_throughput_fps: float
    max_concurrent_users: int
    
    # Software environment
    cuda_version: str
    python_version: str
    ros_distribution: str
    
    # Network parameters
    network_bandwidth_mbps: int
    network_latency_ms: float
    
    # Safety parameters
    safety_factor: float  # Multiplier for safety margins (1.0 = normal, >1.0 = more conservative)

# Configuration for each tier
TIER_CONFIGS = {
    0: HardwareTierConfig(
        tier=0,
        gpu_model="Cloud GPU (Variable)",
        gpu_vram_gb=16,  # Minimum recommended
        cpu_cores=8,
        system_ram_gb=32,
        storage_type="SSD",
        max_inference_latency_ms=200.0,
        max_vla_throughput_fps=5.0,
        max_concurrent_users=10,
        cuda_version="12.6",
        python_version="3.10",
        ros_distribution="Iron",
        network_bandwidth_mbps=1000,
        network_latency_ms=20.0,
        safety_factor=1.0
    ),
    1: HardwareTierConfig(
        tier=1,
        gpu_model="RTX 4090",
        gpu_vram_gb=24,
        cpu_cores=16,
        system_ram_gb=64,
        storage_type="NVMe",
        max_inference_latency_ms=90.0,
        max_vla_throughput_fps=10.0,
        max_concurrent_users=5,
        cuda_version="12.6",
        python_version="3.10",
        ros_distribution="Iron",
        network_bandwidth_mbps=1000,
        network_latency_ms=1.0,
        safety_factor=1.0
    ),
    2: HardwareTierConfig(
        tier=2,
        gpu_model="Jetson Orin NX 16GB",
        gpu_vram_gb=16,
        cpu_cores=8,
        system_ram_gb=16,
        storage_type="NVMe",
        max_inference_latency_ms=220.0,
        max_vla_throughput_fps=3.0,
        max_concurrent_users=1,
        cuda_version="12.6",
        python_version="3.10",
        ros_distribution="Iron",
        network_bandwidth_mbps=100,
        network_latency_ms=5.0,
        safety_factor=1.2  # More conservative for edge deployment
    ),
    3: HardwareTierConfig(
        tier=3,
        gpu_model="Isaac Compatible Platform",
        gpu_vram_gb=8,
        cpu_cores=4,
        system_ram_gb=16,
        storage_type="SSD",
        max_inference_latency_ms=500.0,
        max_vla_throughput_fps=1.0,
        max_concurrent_users=1,
        cuda_version="12.6",
        python_version="3.10",
        ros_distribution="Iron",
        network_bandwidth_mbps=100,
        network_latency_ms=10.0,
        safety_factor=1.5  # More conservative for real robot deployment
    ),
    4: HardwareTierConfig(
        tier=4,
        gpu_model="On-Robot Compute (Limited)",
        gpu_vram_gb=4,
        cpu_cores=4,
        system_ram_gb=8,
        storage_type="eMMC",
        max_inference_latency_ms=1000.0,
        max_vla_throughput_fps=0.5,
        max_concurrent_users=1,
        cuda_version="12.6",
        python_version="3.10",
        ros_distribution="Iron",
        network_bandwidth_mbps=50,
        network_latency_ms=50.0,
        safety_factor=2.0  # Most conservative for human-robot interaction
    )
}

class ConfigManager:
    """
    Configuration manager for Module 4 VLA system
    Handles configuration for different hardware tiers and deployment scenarios
    """
    
    def __init__(self, config_file: Optional[str] = None, default_tier: int = 1):
        """
        Initialize configuration manager
        
        Args:
            config_file: Path to custom configuration file (optional)
            default_tier: Default hardware tier to use (0-4)
        """
        self.default_tier = default_tier
        self.current_tier = default_tier
        self.config_file = config_file
        self.config = self._load_configuration()
        
        logger.info(f"Configuration manager initialized for tier {default_tier}")
    
    def _load_configuration(self) -> Dict[str, Any]:
        """
        Load configuration from file or use defaults
        """
        if self.config_file and os.path.exists(self.config_file):
            return self._load_from_file(self.config_file)
        else:
            return self._get_default_config(self.default_tier)
    
    def _load_from_file(self, file_path: str) -> Dict[str, Any]:
        """
        Load configuration from a file (JSON or YAML)
        
        Args:
            file_path: Path to configuration file
            
        Returns:
            Dictionary with configuration
        """
        try:
            with open(file_path, 'r') as f:
                if file_path.endswith('.json'):
                    config = json.load(f)
                elif file_path.endswith(('.yml', '.yaml')):
                    config = yaml.safe_load(f)
                else:
                    raise ValueError(f"Unsupported config file format: {file_path}")
            
            logger.info(f"Configuration loaded from file: {file_path}")
            return config
        except Exception as e:
            logger.error(f"Error loading config from {file_path}: {e}")
            logger.info("Falling back to default configuration")
            return self._get_default_config(self.default_tier)
    
    def _get_default_config(self, tier: int) -> Dict[str, Any]:
        """
        Get default configuration for a specific tier
        
        Args:
            tier: Hardware tier (0-4)
            
        Returns:
            Dictionary with configuration for the tier
        """
        if tier not in TIER_CONFIGS:
            logger.warning(f"Unknown tier {tier}, using tier 1 as default")
            tier = 1
        
        config = asdict(TIER_CONFIGS[tier])
        
        # Add VLA-specific parameters
        config.update({
            'current_tier': tier,
            'model_precision': 'float16' if tier < 3 else 'int8',  # Use lower precision on edge
            'batch_size': 1 if tier >= 2 else 4,  # Smaller batches on edge
            'max_sequence_length': 512 if tier >= 3 else 2048,  # Shorter sequences on robot
            'action_smoothing': tier > 2,  # Enable smoothing for real robot
            'safety_checks_enabled': tier >= 2,  # Enable safety checks on real hardware
            'logging_level': 'INFO' if tier < 3 else 'WARNING'  # Less logging on resource-constrained
        })
        
        return config
    
    def get_current_config(self) -> Dict[str, Any]:
        """
        Get the current system configuration
        
        Returns:
            Dictionary with current configuration
        """
        return self.config.copy()
    
    def get_tier_config(self, tier: Optional[int] = None) -> HardwareTierConfig:
        """
        Get configuration for a specific hardware tier
        
        Args:
            tier: Hardware tier (0-4). If None, uses current tier.
            
        Returns:
            HardwareTierConfig object
        """
        tier = tier or self.current_tier
        if tier not in TIER_CONFIGS:
            logger.warning(f"Tier {tier} not available, using tier 1")
            tier = 1
        
        return TIER_CONFIGS[tier]
    
    def update_tier(self, tier: int) -> bool:
        """
        Update the current hardware tier
        
        Args:
            tier: New hardware tier (0-4)
            
        Returns:
            True if successful, False otherwise
        """
        if tier not in TIER_CONFIGS:
            logger.error(f"Invalid tier {tier}, must be 0-4")
            return False
        
        old_tier = self.current_tier
        self.current_tier = tier
        self.config = self._get_default_config(tier)
        
        logger.info(f"Updated hardware tier from {old_tier} to {tier}")
        return True
    
    def get_performance_parameter(self, param_name: str, tier: Optional[int] = None) -> Union[float, int]:
        """
        Get a performance parameter for a specific tier
        
        Args:
            param_name: Name of the parameter to get
            tier: Hardware tier (0-4). If None, uses current tier.
            
        Returns:
            Value of the parameter
        """
        tier = tier or self.current_tier
        tier_config = self.get_tier_config(tier)
        
        if hasattr(tier_config, param_name):
            return getattr(tier_config, param_name)
        else:
            raise ValueError(f"Parameter {param_name} not found in tier {tier} config")
    
    def is_suitable_for_tier(self, required_vram_gb: int, required_compute: str = "standard") -> bool:
        """
        Check if current configuration is suitable for requirements
        
        Args:
            required_vram_gb: Required VRAM in GB
            required_compute: Required compute level ("low", "standard", "high")
            
        Returns:
            True if configuration is suitable, False otherwise
        """
        current_config = self.get_tier_config(self.current_tier)
        
        # Check VRAM
        if current_config.gpu_vram_gb < required_vram_gb:
            logger.warning(f"Not enough VRAM: require {required_vram_gb}GB, have {current_config.gpu_vram_gb}GB")
            return False
        
        # For this simple check, we just ensure VRAM is sufficient
        # Additional compute checks could be added here
        return True
    
    def get_optimal_model_settings(self) -> Dict[str, Any]:
        """
        Get optimal model settings for current tier
        
        Returns:
            Dictionary with optimal model settings
        """
        current_config = self.get_tier_config(self.current_tier)
        
        settings = {
            'precision': 'float16' if current_config.gpu_vram_gb >= 16 else 'int8',
            'batch_size': 1 if current_config.gpu_vram_gb < 16 else 4,
            'max_new_tokens': 16 if self.current_tier >= 3 else 32,
            'temperature': 0.1 if self.current_tier >= 3 else 0.0,  # Slightly more random for edge
        }
        
        return settings
    
    def save_config(self, file_path: str, format: str = 'json'):
        """
        Save current configuration to a file
        
        Args:
            file_path: Path to save configuration
            format: Format to save in ('json' or 'yaml')
        """
        try:
            config_to_save = {
                'current_tier': self.current_tier,
                'default_tier': self.default_tier,
                'parameters': self.config
            }
            
            if format.lower() == 'json':
                with open(file_path, 'w') as f:
                    json.dump(config_to_save, f, indent=2)
            elif format.lower() in ['yaml', 'yml']:
                with open(file_path, 'w') as f:
                    yaml.dump(config_to_save, f, default_flow_style=False)
            else:
                raise ValueError(f"Unsupported format: {format}")
            
            logger.info(f"Configuration saved to {file_path}")
        except Exception as e:
            logger.error(f"Error saving config to {file_path}: {e}")
    
    def get_safety_factor(self, tier: Optional[int] = None) -> float:
        """
        Get the safety factor for a tier, which affects safety margins
        
        Args:
            tier: Hardware tier (0-4). If None, uses current tier.
            
        Returns:
            Safety factor multiplier
        """
        tier = tier or self.current_tier
        tier_config = self.get_tier_config(tier)
        return tier_config.safety_factor
    
    def get_vla_pipeline_config(self) -> Dict[str, Any]:
        """
        Get configuration specific to the VLA pipeline for current tier
        
        Returns:
            Dictionary with VLA-specific configuration
        """
        return {
            # Model configuration
            'model_precision': self.config.get('model_precision', 'float16'),
            'batch_size': self.config.get('batch_size', 1),
            'max_sequence_length': self.config.get('max_sequence_length', 512),
            
            # Performance parameters
            'max_inference_latency': self.get_performance_parameter('max_inference_latency_ms'),
            'max_throughput': self.get_performance_parameter('max_vla_throughput_fps'),
            
            # Operational parameters
            'action_smoothing': self.config.get('action_smoothing', False),
            'safety_checks_enabled': self.config.get('safety_checks_enabled', True),
            'logging_level': self.config.get('logging_level', 'INFO'),
            
            # Safety parameters
            'safety_factor': self.get_safety_factor()
        }


class GlobalConfig:
    """
    Global configuration singleton for the entire Module 4 system
    """
    _instance = None
    _initialized = False
    
    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(GlobalConfig, cls).__new__(cls)
        return cls._instance
    
    def __init__(self, config_file: Optional[str] = None, default_tier: int = 1):
        if self._initialized:
            return
            
        self.config_manager = ConfigManager(config_file, default_tier)
        self._initialized = True
    
    def get_config(self) -> Dict[str, Any]:
        """Get current configuration"""
        return self.config_manager.get_current_config()
    
    def get_vla_config(self) -> Dict[str, Any]:
        """Get VLA-specific configuration"""
        return self.config_manager.get_vla_pipeline_config()
    
    def update_tier(self, tier: int) -> bool:
        """Update hardware tier"""
        return self.config_manager.update_tier(tier)
    
    def get_current_tier(self) -> int:
        """Get current hardware tier"""
        return self.config_manager.current_tier
    
    def get_safety_factor(self) -> float:
        """Get safety factor for current tier"""
        return self.config_manager.get_safety_factor()


# Convenience functions
def get_global_config() -> GlobalConfig:
    """Get the global configuration instance"""
    return GlobalConfig()

def get_current_vla_config() -> Dict[str, Any]:
    """Get the current VLA-specific configuration"""
    return get_global_config().get_vla_config()

def set_hardware_tier(tier: int) -> bool:
    """Update the hardware tier for the global configuration"""
    return get_global_config().update_tier(tier)

def get_current_tier() -> int:
    """Get the current hardware tier"""
    return get_global_config().get_current_tier()


if __name__ == "__main__":
    # Example usage
    print("Testing Configuration Management...")
    
    # Initialize config manager with default tier (1 - RTX 4090)
    config_mgr = ConfigManager(default_tier=1)
    print("Configuration manager initialized successfully!")
    
    # Get current configuration
    current_config = config_mgr.get_current_config()
    print(f"Current tier: {current_config['current_tier']}")
    
    # Get VLA-specific configuration
    vla_config = config_mgr.get_vla_pipeline_config()
    print(f"VLA precision: {vla_config['model_precision']}")
    print(f"VLA batch size: {vla_config['batch_size']}")
    
    # Test updating to a different tier (e.g., Jetson Orin NX)
    success = config_mgr.update_tier(2)
    print(f"Updated to tier 2 (Jetson Orin NX): {success}")
    
    if success:
        new_vla_config = config_mgr.get_vla_pipeline_config()
        print(f"New VLA precision for tier 2: {new_vla_config['model_precision']}")
        print(f"New VLA batch size for tier 2: {new_vla_config['batch_size']}")
        print(f"New safety factor: {new_vla_config['safety_factor']}")
    
    # Test global configuration singleton
    global_config = get_global_config()
    print(f"Global config tier: {global_config.get_current_tier()}")
    
    # Test setting tier through global config
    set_hardware_tier(1)  # Back to RTX 4090
    print(f"Global config tier after update: {get_current_tier()}")
    
    # Get VLA config through convenience function
    vla_cfg = get_current_vla_config()
    print(f"Current VLA max latency: {vla_cfg['max_inference_latency']}ms")
    
    print("Configuration Management test completed.")