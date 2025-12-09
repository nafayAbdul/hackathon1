"""
Action Space Conversion Utilities
Listing 16.3: Action space conversion utilities
"""
import numpy as np
from typing import List, Dict, Tuple, Optional, Union
from dataclasses import dataclass


@dataclass
class ActionSpaceConfig:
    """
    Configuration for action space conversion
    """
    # VLA model action space properties
    vla_action_dim: int = 7  # Common dimension for VLA models
    vla_action_min: float = -1.0
    vla_action_max: float = 1.0
    
    # Robot-specific action space properties
    robot_type: str = "athena"
    robot_action_dim: int = 24  # 24 DoF for athena humanoid
    
    # Joint limits (example values, would be robot-specific)
    joint_limits_min: List[float] = None
    joint_limits_max: List[float] = None
    
    # Mapping from VLA action space to robot action space
    action_mapping: List[int] = None  # Indices of robot joints controlled by VLA


def create_athena_config() -> ActionSpaceConfig:
    """
    Create action space configuration for Athena humanoid robot
    
    Returns:
        ActionSpaceConfig with Athena-specific parameters
    """
    # Example configuration for Athena humanoid (24 DoF)
    joint_limits_min = [-2.0] * 24  # Simplified, actual joints have different limits
    joint_limits_max = [2.0] * 24   # Simplified, actual joints have different limits
    
    # Map 7 VLA actions to specific joints (e.g., arm joints)
    action_mapping = [0, 1, 2, 6, 7, 8, 20]  # Example: 3 left arm + 3 right arm + 1 leg joint
    
    return ActionSpaceConfig(
        vla_action_dim=7,
        vla_action_min=-1.0,
        vla_action_max=1.0,
        robot_type="athena",
        robot_action_dim=24,
        joint_limits_min=joint_limits_min,
        joint_limits_max=joint_limits_max,
        action_mapping=action_mapping
    )


def normalize_vla_action(action: np.ndarray, 
                        config: ActionSpaceConfig) -> np.ndarray:
    """
    Normalize VLA action to the expected range
    
    Args:
        action: Raw action from VLA model
        config: Action space configuration
        
    Returns:
        Normalized action within VLA action space bounds
    """
    # Clamp to VLA action bounds
    normalized = np.clip(action, config.vla_action_min, config.vla_action_max)
    return normalized


def denormalize_vla_action(action: np.ndarray,
                          config: ActionSpaceConfig) -> np.ndarray:
    """
    Denormalize action from VLA space to a different range if needed
    
    Args:
        action: Normalized action in VLA space
        config: Action space configuration
        
    Returns:
        Denormalized action
    """
    # For now, just return the action as is
    # In practice, this might transform to another range
    return action


def map_vla_to_robot_action(vla_action: np.ndarray, 
                           config: ActionSpaceConfig,
                           current_robot_state: Optional[np.ndarray] = None) -> np.ndarray:
    """
    Map VLA action to robot action space
    
    Args:
        vla_action: Action from VLA model (e.g., 7-DOF)
        config: Action space configuration
        current_robot_state: Current state of the robot (for relative actions)
        
    Returns:
        Robot action in full joint space
    """
    # First, normalize the VLA action
    normalized_action = normalize_vla_action(vla_action, config)
    
    # Create the full robot action vector
    robot_action = np.zeros(config.robot_action_dim)
    
    # If we have a mapping, use it to place the VLA actions in the right joints
    if config.action_mapping and len(config.action_mapping) <= len(normalized_action):
        for i, joint_idx in enumerate(config.action_mapping):
            if i < len(normalized_action):
                robot_action[joint_idx] = normalized_action[i]
    else:
        # If no mapping provided, evenly distribute the action
        # (This is a fallback, in practice you'd want specific mappings)
        min_joints = min(config.robot_action_dim, len(normalized_action))
        robot_action[:min_joints] = normalized_action[:min_joints]
    
    # Apply joint limits if specified
    if config.joint_limits_min and config.joint_limits_max:
        robot_action = np.clip(
            robot_action, 
            config.joint_limits_min, 
            config.joint_limits_max
        )
    
    return robot_action


def convert_cartesian_to_joint(cartesian_pose: Tuple[float, float, float, float, float, float],
                              robot_state: np.ndarray,
                              config: ActionSpaceConfig) -> np.ndarray:
    """
    Convert Cartesian pose to joint space (simplified implementation)
    
    Args:
        cartesian_pose: (x, y, z, roll, pitch, yaw) of end-effector
        robot_state: Current joint positions
        config: Action space configuration
        
    Returns:
        Joint position changes to achieve the desired pose
    """
    # This is a highly simplified version
    # A real implementation would use inverse kinematics
    
    # Just return a small change for demonstration
    joint_delta = np.random.uniform(-0.1, 0.1, size=config.robot_action_dim)
    return joint_delta


def convert_joint_to_cartesian(joint_positions: np.ndarray,
                              config: ActionSpaceConfig) -> Tuple[float, float, float, float, float, float]:
    """
    Convert joint space to Cartesian pose (simplified implementation)
    
    Args:
        joint_positions: Current joint positions
        config: Action space configuration
        
    Returns:
        (x, y, z, roll, pitch, yaw) of end-effector
    """
    # This is a highly simplified version
    # A real implementation would use forward kinematics
    
    # Just return a dummy pose for demonstration
    return (0.5, 0.0, 0.8, 0.0, 0.0, 0.0)


def scale_action_space(action: np.ndarray, 
                      from_range: Tuple[float, float],
                      to_range: Tuple[float, float]) -> np.ndarray:
    """
    Scale an action from one range to another
    
    Args:
        action: Input action array
        from_range: Original range (min, max)
        to_range: Target range (min, max)
        
    Returns:
        Scaled action array
    """
    from_min, from_max = from_range
    to_min, to_max = to_range
    
    # Normalize to [0, 1] range
    normalized = (action - from_min) / (from_max - from_min)
    
    # Scale to target range
    scaled = normalized * (to_max - to_min) + to_min
    
    return scaled


def validate_action_space(action: np.ndarray, config: ActionSpaceConfig) -> Dict[str, bool]:
    """
    Validate an action against various constraints
    
    Args:
        action: Action array to validate
        config: Action space configuration
        
    Returns:
        Dictionary with validation results
    """
    results = {
        "finite_values": np.all(np.isfinite(action)),
        "within_vla_bounds": True,
        "within_robot_bounds": True,
        "correct_dimension": True
    }
    
    # Check dimensions
    if len(action) != config.robot_action_dim:
        results["correct_dimension"] = False
    
    # Check VLA bounds if we only consider the VLA dimensions
    if config.action_mapping:
        vla_portion = action[config.action_mapping]
        results["within_vla_bounds"] = np.all(
            (vla_portion >= config.vla_action_min) & 
            (vla_portion <= config.vla_action_max)
        )
    else:
        vla_portion_size = min(len(action), config.vla_action_dim)
        vla_portion = action[:vla_portion_size]
        results["within_vla_bounds"] = np.all(
            (vla_portion >= config.vla_action_min) & 
            (vla_portion <= config.vla_action_max)
        )
    
    # Check robot bounds
    if config.joint_limits_min and config.joint_limits_max:
        results["within_robot_bounds"] = np.all(
            (action >= config.joint_limits_min) & 
            (action <= config.joint_limits_max)
        )
    
    return results


class ActionSpaceConverter:
    """
    A class to handle various action space conversions
    """
    
    def __init__(self, config: ActionSpaceConfig = None):
        """
        Initialize the action space converter
        
        Args:
            config: Action space configuration
        """
        self.config = config or create_athena_config()
    
    def convert(self, vla_action: np.ndarray, 
                method: str = "direct",
                current_state: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Convert VLA action to robot action using specified method
        
        Args:
            vla_action: Action from VLA model
            method: Conversion method ("direct", "relative", "scaled")
            current_state: Current robot state (for methods that need it)
            
        Returns:
            Converted robot action
        """
        if method == "direct":
            return map_vla_to_robot_action(vla_action, self.config, current_state)
        elif method == "relative":
            if current_state is None:
                raise ValueError("Current state required for relative method")
            # This would add the VLA action to the current state
            robot_action = map_vla_to_robot_action(vla_action, self.config)
            # Only apply to mapped joints
            result = current_state.copy()
            if self.config.action_mapping:
                for i, joint_idx in enumerate(self.config.action_mapping):
                    if i < len(robot_action):
                        result[joint_idx] += robot_action[i]
            else:
                min_joints = min(len(result), len(robot_action))
                result[:min_joints] += robot_action[:min_joints]
            return result
        elif method == "scaled":
            # Scale the action to use full range
            scaled_vla = scale_action_space(
                vla_action, 
                (self.config.vla_action_min, self.config.vla_action_max),
                (self.config.vla_action_min, self.config.vla_action_max)  # Same range for now
            )
            return map_vla_to_robot_action(scaled_vla, self.config, current_state)
        else:
            raise ValueError(f"Unknown conversion method: {method}")
    
    def batch_convert(self, vla_actions: List[np.ndarray], 
                     method: str = "direct",
                     current_states: Optional[List[np.ndarray]] = None) -> List[np.ndarray]:
        """
        Convert multiple VLA actions to robot actions
        
        Args:
            vla_actions: List of VLA actions
            method: Conversion method
            current_states: List of current robot states (for relative methods)
            
        Returns:
            List of converted robot actions
        """
        results = []
        for i, action in enumerate(vla_actions):
            current_state = current_states[i] if current_states else None
            converted = self.convert(action, method, current_state)
            results.append(converted)
        return results


def example_usage():
    """
    Example usage of action space conversion utilities
    """
    print("Action Space Conversion Examples:")
    
    # Create configuration for Athena robot
    config = create_athena_config()
    print(f"Created configuration for {config.robot_type} robot")
    print(f"VLA action dim: {config.vla_action_dim}, Robot action dim: {config.robot_action_dim}")
    
    # Example 1: Direct mapping
    print("\n1. Direct mapping example:")
    vla_action = np.array([0.5, -0.3, 0.8, 0.1, -0.7, 0.4, 0.2])
    robot_action = map_vla_to_robot_action(vla_action, config)
    print(f"VLA action: {vla_action}")
    print(f"Robot action shape: {robot_action.shape}")
    print(f"First 6 robot joints: {robot_action[:6]}")
    
    # Example 2: Using the converter class
    print("\n2. Using ActionSpaceConverter:")
    converter = ActionSpaceConverter(config)
    robot_action2 = converter.convert(vla_action, method="direct")
    print(f"Converted using direct method: {robot_action2[:6]} (first 6 joints)")
    
    # Example 3: Validation
    print("\n3. Validation example:")
    validation_results = validate_action_space(robot_action, config)
    for check, result in validation_results.items():
        print(f"  {check}: {result}")
    
    # Example 4: Range scaling
    print("\n4. Range scaling example:")
    raw_action = np.array([-2.0, 1.5, 0.0, 3.0, -1.0])
    # Scale from a different range to VLA range
    scaled_action = scale_action_space(raw_action, (-3.0, 3.0), (-1.0, 1.0))
    print(f"Raw action: {raw_action}")
    print(f"Scaled action: {scaled_action}")
    
    # Example 5: Relative action (requires current state)
    print("\n5. Relative action conversion:")
    current_state = np.zeros(config.robot_action_dim)  # Default joint positions
    relative_action = converter.convert(vla_action, method="relative", current_state=current_state)
    print(f"Relative action applied to current state: {relative_action[:6]} (first 6 joints)")


if __name__ == "__main__":
    example_usage()