"""
Common Data Structures for Vision-Language-Action Systems

This module defines common data structures used across vision, language, and action components
of the VLA system.
"""
from dataclasses import dataclass
from typing import List, Dict, Optional, Any, Union, Tuple
import numpy as np
from enum import Enum


class HardwareTier(Enum):
    """
    Hardware tier enumeration for different deployment levels
    """
    TIER_0_CLOUD = "Tier 0 (Cloud)"
    TIER_1_SIMULATION = "Tier 1 (Simulation)"
    TIER_2_EDGE_GPU = "Tier 2 (Edge GPU - Jetson Orin NX 16GB)"
    TIER_3_ISAAC_PLATFORM = "Tier 3 (NVIDIA Isaac Platform)"
    TIER_4_REAL_HUMANOID = "Tier 4 (Real Humanoid Hardware)"


class VLAComponent(Enum):
    """
    Components of the VLA system
    """
    VISION = "vision"
    LANGUAGE = "language"
    ACTION = "action"
    FUSION = "fusion"


@dataclass
class VisionInput:
    """
    Data structure for vision input to VLA models
    """
    image: np.ndarray  # RGB image data
    camera_intrinsics: Optional[np.ndarray] = None  # Camera intrinsics matrix
    camera_extrinsics: Optional[np.ndarray] = None  # Camera extrinsics matrix
    depth_image: Optional[np.ndarray] = None  # Depth information if available
    objects: Optional[List[Dict]] = None  # Detected objects with bounding boxes
    timestamp: Optional[float] = None  # Timestamp of capture


@dataclass
class LanguageInput:
    """
    Data structure for language input to VLA models
    """
    text: str  # Natural language instruction
    embedding: Optional[np.ndarray] = None  # Text embedding vector
    intent: Optional[str] = None  # Parsed intent
    entities: Optional[Dict[str, Any]] = None  # Extracted entities
    confidence: Optional[float] = None  # Confidence in the input
    timestamp: Optional[float] = None  # Timestamp of input


@dataclass
class ActionOutput:
    """
    Data structure for action output from VLA models
    """
    joint_positions: List[float]  # Desired joint positions
    joint_velocities: Optional[List[float]] = None  # Desired joint velocities
    cartesian_pose: Optional[Tuple[float, float, float, float, float, float]] = None  # x, y, z, roll, pitch, yaw
    gripper_action: Optional[float] = None  # Gripper control (0=open, 1=closed)
    confidence: Optional[float] = None  # Confidence in the action
    execution_time: Optional[float] = None  # Expected execution time


@dataclass
class VLAPrediction:
    """
    Complete prediction output from VLA model
    """
    vision_input: VisionInput
    language_input: LanguageInput
    action_output: ActionOutput
    attention_weights: Optional[np.ndarray] = None  # Attention weights for interpretability
    execution_log: Optional[List[str]] = None  # Log of execution steps
    success_probability: Optional[float] = None  # Estimated success probability


@dataclass
class RobotState:
    """
    Current state of the robot
    """
    joint_positions: List[float]
    joint_velocities: List[float]
    joint_efforts: List[float]
    cartesian_pose: Optional[Tuple[float, float, float, float, float, float]] = None
    sensors_data: Optional[Dict[str, Any]] = None
    timestamp: Optional[float] = None


@dataclass
class TaskPlan:
    """
    Planned sequence of actions for a task
    """
    task_description: str
    action_sequence: List[ActionOutput]
    priority: int = 1  # Higher number means higher priority
    estimated_duration: Optional[float] = None
    success_criteria: Optional[List[str]] = None
    dependencies: Optional[List[str]] = None  # Other tasks this task depends on


@dataclass
class PerceptionResult:
    """
    Result of perception processing
    """
    detected_objects: List[Dict]  # Objects with properties like position, type, etc.
    scene_description: str  # Textual description of the scene
    confidence_map: Optional[np.ndarray] = None  # Confidence map for detections
    segmentation_mask: Optional[np.ndarray] = None  # Segmentation mask
    timestamp: Optional[float] = None


@dataclass
class SafetyParameters:
    """
    Safety parameters for action execution
    """
    max_velocity: float
    max_torque: float
    safety_margin: float
    emergency_stop_threshold: float
    collision_threshold: float
    joint_limit_margin: float


@dataclass
class VLAConfig:
    """
    Configuration for VLA model
    """
    model_name: str
    device: str
    precision: str
    image_size: Tuple[int, int]
    action_space_dim: int
    hardware_tier: HardwareTier
    safety_params: SafetyParameters
    fusion_method: str  # How vision and language are fused


@dataclass
class PerformanceMetrics:
    """
    Performance metrics for VLA system evaluation
    """
    inference_time: float  # Time for VLA inference
    action_success_rate: float  # Success rate of actions
    language_understanding_accuracy: float  # Accuracy of language understanding
    vision_detection_accuracy: float  # Accuracy of vision detection
    end_to_end_latency: float  # Total system latency
    vram_usage: Optional[float] = None  # VRAM usage in GB
    cpu_usage: Optional[float] = None  # CPU usage percentage
    power_consumption: Optional[float] = None  # Power consumption in watts


@dataclass
class SystemStatus:
    """
    Overall system status
    """
    component_status: Dict[VLAComponent, str]  # Status per VLA component
    hardware_tier: HardwareTier
    current_task: Optional[str] = None
    battery_level: Optional[float] = None
    temperature: Optional[Dict[str, float]] = None  # Component temperatures
    performance_metrics: Optional[PerformanceMetrics] = None
    timestamp: Optional[float] = None


@dataclass
class CalibrationData:
    """
    Calibration data for mapping VLA outputs to robot actions
    """
    vision_to_robot_calibration: np.ndarray  # Transformation matrix for vision-to-robot mapping
    action_space_mapping: Dict[str, Any]  # Mapping from VLA action space to robot action space
    gripper_calibration: Dict[str, float]  # Gripper-specific calibration parameters
    kinematic_params: Dict[str, Any]  # Kinematic parameters for the robot
    timestamp: Optional[float] = None


@dataclass
class ErrorLog:
    """
    Error logging structure
    """
    error_type: str
    error_message: str
    component: VLAComponent
    timestamp: float
    severity: str  # 'low', 'medium', 'high', 'critical'
    context: Optional[Dict[str, Any]] = None  # Additional context about the error


# Utility functions for data structure operations
def merge_vision_language_inputs(vision_input: VisionInput, 
                                language_input: LanguageInput) -> Dict[str, Any]:
    """
    Merge vision and language inputs into a format suitable for VLA models
    
    Args:
        vision_input: Vision input data
        language_input: Language input data
        
    Returns:
        Merged input in dictionary format
    """
    merged_input = {
        "image": vision_input.image,
        "text": language_input.text,
        "camera_intrinsics": vision_input.camera_intrinsics,
        "camera_extrinsics": vision_input.camera_extrinsics,
        "depth_image": vision_input.depth_image,
        "objects": vision_input.objects,
        "embedding": language_input.embedding,
        "intent": language_input.intent,
        "entities": language_input.entities,
        "confidence": language_input.confidence
    }
    return merged_input


def validate_vla_prediction(prediction: VLAPrediction) -> bool:
    """
    Validate the structure of a VLA prediction
    
    Args:
        prediction: VLAPrediction object to validate
        
    Returns:
        True if valid, False otherwise
    """
    # Check that required fields are present and valid
    if not isinstance(prediction.vision_input, VisionInput):
        return False
    if not isinstance(prediction.language_input, LanguageInput):
        return False
    if not isinstance(prediction.action_output, ActionOutput):
        return False
    
    # Check that image and text are present
    if prediction.vision_input.image is None:
        return False
    if not prediction.language_input.text:
        return False
    
    # Check joint positions are valid
    if not prediction.action_output.joint_positions:
        return False
    
    # Check all joint positions are finite
    for pos in prediction.action_output.joint_positions:
        if not np.isfinite(pos):
            return False
    
    return True


def create_default_safety_params() -> SafetyParameters:
    """
    Create default safety parameters
    
    Returns:
        SafetyParameters object with default values
    """
    return SafetyParameters(
        max_velocity=0.5,
        max_torque=50.0,
        safety_margin=0.1,
        emergency_stop_threshold=0.95,
        collision_threshold=0.05,
        joint_limit_margin=0.05
    )


def generate_task_plan_from_command(command: str) -> TaskPlan:
    """
    Generate a simple task plan from a natural language command
    
    Args:
        command: Natural language command
        
    Returns:
        TaskPlan object with basic structure
    """
    # This is a simplified version - in practice, this would use more sophisticated NLP
    return TaskPlan(
        task_description=command,
        action_sequence=[],
        success_criteria=[f"Task '{command}' completed successfully"]
    )