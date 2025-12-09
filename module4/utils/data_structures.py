"""
Data structures for vision, language, and action components in Module 4
Defines standard formats for exchanging information between VLA system components
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Union, Tuple
import numpy as np
from enum import Enum
import json
from datetime import datetime
import uuid


class ActionType(Enum):
    """
    Enumeration of possible action types in the VLA system
    """
    JOINT_POSITIONS = "joint_positions"
    JOINT_VELOCITIES = "joint_velocities"
    JOINT_EFFORTS = "joint_efforts"
    CARTESIAN_POSE = "cartesian_pose"
    CARTESIAN_TWIST = "cartesian_twist"
    GRIPPER_COMMAND = "gripper_command"
    BASE_MOTION = "base_motion"
    COMPOSITE = "composite"


class TaskType(Enum):
    """
    Enumeration of different task types for the VLA system
    """
    MANIPULATION = "manipulation"
    NAVIGATION = "navigation"
    INSPECTION = "inspection"
    ASSEMBLY = "assembly"
    HANDOVER = "handover"
    STORAGE = "storage"


@dataclass
class JointState:
    """
    Represents the state of robot joints
    """
    positions: List[float]
    velocities: Optional[List[float]] = None
    efforts: Optional[List[float]] = None
    timestamp: Optional[float] = None
    joint_names: Optional[List[str]] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()
        
        if self.joint_names is None:
            self.joint_names = [f"joint_{i}" for i in range(len(self.positions))]
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert JointState to dictionary representation"""
        return {
            'positions': self.positions,
            'velocities': self.velocities,
            'efforts': self.efforts,
            'timestamp': self.timestamp,
            'joint_names': self.joint_names
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'JointState':
        """Create JointState from dictionary representation"""
        return cls(
            positions=data['positions'],
            velocities=data.get('velocities'),
            efforts=data.get('efforts'),
            timestamp=data.get('timestamp'),
            joint_names=data.get('joint_names')
        )


@dataclass
class CartesianPose:
    """
    Represents a Cartesian pose (position and orientation)
    """
    position: List[float]  # [x, y, z] in meters
    orientation: List[float]  # [x, y, z, w] as quaternion
    reference_frame: str = "base_link"
    timestamp: Optional[float] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()
        
        # Validate input dimensions
        if len(self.position) != 3:
            raise ValueError(f"Position must have 3 elements, got {len(self.position)}")
        
        if len(self.orientation) != 4:
            raise ValueError(f"Orientation must have 4 elements (quaternion), got {len(self.orientation)}")
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert CartesianPose to dictionary representation"""
        return {
            'position': self.position,
            'orientation': self.orientation,
            'reference_frame': self.reference_frame,
            'timestamp': self.timestamp
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'CartesianPose':
        """Create CartesianPose from dictionary representation"""
        return cls(
            position=data['position'],
            orientation=data['orientation'],
            reference_frame=data.get('reference_frame', 'base_link'),
            timestamp=data.get('timestamp')
        )


@dataclass
class VisionObservation:
    """
    Represents visual observation from robot sensors
    """
    image_data: Optional[bytes] = None  # Raw image bytes
    depth_data: Optional[List[List[float]]] = None  # Depth map
    camera_intrinsics: Optional[List[float]] = field(default_factory=list)  # [fx, fy, cx, cy]
    camera_extrinsics: Optional[List[List[float]]] = field(default_factory=list)  # 4x4 transformation matrix
    objects: Optional[List[Dict[str, Any]]] = field(default_factory=list)  # Detected objects
    segmentation_mask: Optional[bytes] = None  # Segmentation mask as bytes
    timestamp: Optional[float] = None
    source_camera: str = "default_camera"
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert VisionObservation to dictionary representation"""
        # Note: image_data and segmentation_mask are not serialized due to binary nature
        return {
            'depth_data': self.depth_data,
            'camera_intrinsics': self.camera_intrinsics,
            'camera_extrinsics': self.camera_extrinsics,
            'objects': self.objects,
            'timestamp': self.timestamp,
            'source_camera': self.source_camera
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'VisionObservation':
        """Create VisionObservation from dictionary representation"""
        return cls(
            depth_data=data.get('depth_data'),
            camera_intrinsics=data.get('camera_intrinsics', []),
            camera_extrinsics=data.get('camera_extrinsics', []),
            objects=data.get('objects', []),
            timestamp=data.get('timestamp'),
            source_camera=data.get('source_camera', 'default_camera')
        )


@dataclass
class LanguageInstruction:
    """
    Represents a natural language instruction for the VLA system
    """
    text: str
    language: str = "en"
    confidence: float = 1.0
    timestamp: Optional[float] = None
    source: str = "user"  # "user", "system", "generated"
    intent: Optional[str] = None  # Parsed intent
    entities: Optional[Dict[str, str]] = field(default_factory=dict)  # Parsed entities
    original_command: Optional[str] = None  # Original voice command if speech
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()
        
        if self.original_command is None:
            self.original_command = self.text
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert LanguageInstruction to dictionary representation"""
        return {
            'text': self.text,
            'language': self.language,
            'confidence': self.confidence,
            'timestamp': self.timestamp,
            'source': self.source,
            'intent': self.intent,
            'entities': self.entities,
            'original_command': self.original_command
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'LanguageInstruction':
        """Create LanguageInstruction from dictionary representation"""
        return cls(
            text=data['text'],
            language=data.get('language', 'en'),
            confidence=data.get('confidence', 1.0),
            timestamp=data.get('timestamp'),
            source=data.get('source', 'user'),
            intent=data.get('intent'),
            entities=data.get('entities', {}),
            original_command=data.get('original_command', data['text'])
        )


@dataclass
class VLAPrediction:
    """
    Represents a prediction from the Vision-Language-Action model
    """
    action: List[float]  # Raw action values from model
    action_type: ActionType
    confidence: float = 1.0
    visual_features: Optional[List[float]] = None  # High-level visual features
    language_features: Optional[List[float]] = None  # High-level language features
    multimodal_features: Optional[List[float]] = None  # Combined features
    raw_output: Optional[str] = None  # Raw string output from model
    processed_output: Optional[List[float]] = None  # Processed robot-appropriate action
    timestamp: Optional[float] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert VLAPrediction to dictionary representation"""
        return {
            'action': self.action,
            'action_type': self.action_type.value,
            'confidence': self.confidence,
            'visual_features': self.visual_features,
            'language_features': self.language_features,
            'multimodal_features': self.multimodal_features,
            'raw_output': self.raw_output,
            'processed_output': self.processed_output,
            'timestamp': self.timestamp
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'VLAPrediction':
        """Create VLAPrediction from dictionary representation"""
        return cls(
            action=data['action'],
            action_type=ActionType(data['action_type']),
            confidence=data.get('confidence', 1.0),
            visual_features=data.get('visual_features'),
            language_features=data.get('language_features'),
            multimodal_features=data.get('multimodal_features'),
            raw_output=data.get('raw_output'),
            processed_output=data.get('processed_output'),
            timestamp=data.get('timestamp')
        )


@dataclass
class RobotAction:
    """
    Represents an action to be executed on the robot
    """
    action_type: ActionType
    values: List[float]
    duration: Optional[float] = None  # Expected execution time in seconds
    safety_limits: Optional[Dict[str, float]] = field(default_factory=dict)  # Safety parameters
    task_id: Optional[str] = None  # Associated task ID
    timestamp: Optional[float] = None
    execution_status: str = "pending"  # "pending", "in_progress", "completed", "failed"
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()
        
        if self.task_id is None:
            self.task_id = str(uuid.uuid4())
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert RobotAction to dictionary representation"""
        return {
            'action_type': self.action_type.value,
            'values': self.values,
            'duration': self.duration,
            'safety_limits': self.safety_limits,
            'task_id': self.task_id,
            'timestamp': self.timestamp,
            'execution_status': self.execution_status
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'RobotAction':
        """Create RobotAction from dictionary representation"""
        return cls(
            action_type=ActionType(data['action_type']),
            values=data['values'],
            duration=data.get('duration'),
            safety_limits=data.get('safety_limits', {}),
            task_id=data.get('task_id'),
            timestamp=data.get('timestamp'),
            execution_status=data.get('execution_status', 'pending')
        )


@dataclass
class TaskSpecification:
    """
    Represents a complete task specification for the VLA system
    """
    task_type: TaskType
    description: str
    language_instruction: LanguageInstruction
    target_objects: List[str] = field(default_factory=list)
    target_locations: List[str] = field(default_factory=list)
    constraints: Optional[Dict[str, Any]] = field(default_factory=dict)  # Task-specific constraints
    success_criteria: Optional[List[str]] = field(default_factory=list)
    priority: int = 1  # 1-5 priority level
    timeout: float = 30.0  # seconds
    timestamp: Optional[float] = None
    task_id: Optional[str] = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()
        
        if self.task_id is None:
            self.task_id = str(uuid.uuid4())
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert TaskSpecification to dictionary representation"""
        return {
            'task_type': self.task_type.value,
            'description': self.description,
            'language_instruction': self.language_instruction.to_dict(),
            'target_objects': self.target_objects,
            'target_locations': self.target_locations,
            'constraints': self.constraints,
            'success_criteria': self.success_criteria,
            'priority': self.priority,
            'timeout': self.timeout,
            'timestamp': self.timestamp,
            'task_id': self.task_id
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TaskSpecification':
        """Create TaskSpecification from dictionary representation"""
        return cls(
            task_type=TaskType(data['task_type']),
            description=data['description'],
            language_instruction=LanguageInstruction.from_dict(data['language_instruction']),
            target_objects=data.get('target_objects', []),
            target_locations=data.get('target_locations', []),
            constraints=data.get('constraints', {}),
            success_criteria=data.get('success_criteria', []),
            priority=data.get('priority', 1),
            timeout=data.get('timeout', 30.0),
            timestamp=data.get('timestamp'),
            task_id=data.get('task_id')
        )


@dataclass
class TaskExecutionState:
    """
    Represents the state of a task execution
    """
    task_id: str
    status: str  # "planning", "executing", "completed", "failed", "cancelled"
    current_step: int = 0
    total_steps: int = 0
    progress: float = 0.0  # 0.0 to 1.0
    current_action: Optional[RobotAction] = None
    execution_history: List[Dict[str, Any]] = field(default_factory=list)
    error_message: Optional[str] = None
    start_time: Optional[float] = None
    end_time: Optional[float] = None
    
    def __post_init__(self):
        if self.start_time is None:
            self.start_time = datetime.now().timestamp()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert TaskExecutionState to dictionary representation"""
        return {
            'task_id': self.task_id,
            'status': self.status,
            'current_step': self.current_step,
            'total_steps': self.total_steps,
            'progress': self.progress,
            'current_action': self.current_action.to_dict() if self.current_action else None,
            'execution_history': self.execution_history,
            'error_message': self.error_message,
            'start_time': self.start_time,
            'end_time': self.end_time
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'TaskExecutionState':
        """Create TaskExecutionState from dictionary representation"""
        current_action = None
        if data.get('current_action'):
            current_action = RobotAction.from_dict(data['current_action'])
        
        return cls(
            task_id=data['task_id'],
            status=data['status'],
            current_step=data.get('current_step', 0),
            total_steps=data.get('total_steps', 0),
            progress=data.get('progress', 0.0),
            current_action=current_action,
            execution_history=data.get('execution_history', []),
            error_message=data.get('error_message'),
            start_time=data.get('start_time'),
            end_time=data.get('end_time')
        )


@dataclass
class SystemState:
    """
    Represents the complete system state for the VLA system
    """
    robot_joint_state: JointState
    robot_pose: CartesianPose
    latest_vision: VisionObservation
    latest_language: LanguageInstruction
    current_task: Optional[TaskSpecification] = None
    task_execution_state: Optional[TaskExecutionState] = None
    system_uptime: float = 0.0
    active_safety_events: List[Dict[str, Any]] = field(default_factory=list)
    performance_metrics: Dict[str, float] = field(default_factory=dict)
    timestamp: Optional[float] = None
    system_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().timestamp()
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert SystemState to dictionary representation"""
        return {
            'robot_joint_state': self.robot_joint_state.to_dict(),
            'robot_pose': self.robot_pose.to_dict(),
            'latest_vision': self.latest_vision.to_dict(),
            'latest_language': self.latest_language.to_dict(),
            'current_task': self.current_task.to_dict() if self.current_task else None,
            'task_execution_state': self.task_execution_state.to_dict() if self.task_execution_state else None,
            'system_uptime': self.system_uptime,
            'active_safety_events': self.active_safety_events,
            'performance_metrics': self.performance_metrics,
            'timestamp': self.timestamp,
            'system_id': self.system_id
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'SystemState':
        """Create SystemState from dictionary representation"""
        current_task = None
        task_execution_state = None
        
        if data.get('current_task'):
            current_task = TaskSpecification.from_dict(data['current_task'])
        if data.get('task_execution_state'):
            task_execution_state = TaskExecutionState.from_dict(data['task_execution_state'])
        
        return cls(
            robot_joint_state=JointState.from_dict(data['robot_joint_state']),
            robot_pose=CartesianPose.from_dict(data['robot_pose']),
            latest_vision=VisionObservation.from_dict(data['latest_vision']),
            latest_language=LanguageInstruction.from_dict(data['latest_language']),
            current_task=current_task,
            task_execution_state=task_execution_state,
            system_uptime=data.get('system_uptime', 0.0),
            active_safety_events=data.get('active_safety_events', []),
            performance_metrics=data.get('performance_metrics', {}),
            timestamp=data.get('timestamp'),
            system_id=data.get('system_id', str(uuid.uuid4()))
        )


class DataSerializer:
    """
    Utility class for serializing and deserializing VLA data structures
    """
    
    @staticmethod
    def serialize(obj: Any) -> str:
        """
        Serialize a dataclass object to JSON string
        """
        if hasattr(obj, 'to_dict'):
            return json.dumps(obj.to_dict(), default=str)
        else:
            raise TypeError(f"Object {type(obj)} is not serializable with this method")
    
    @staticmethod
    def deserialize(json_str: str, target_class) -> Any:
        """
        Deserialize a JSON string to a dataclass object
        """
        data = json.loads(json_str)
        if hasattr(target_class, 'from_dict'):
            return target_class.from_dict(data)
        else:
            raise TypeError(f"Target class {target_class} is not deserializable with this method")


# Convenience functions for creating common data structures
def create_default_joint_state(num_joints: int = 7) -> JointState:
    """
    Create a default JointState with zeros
    """
    return JointState(positions=[0.0] * num_joints)


def create_default_cartesian_pose() -> CartesianPose:
    """
    Create a default CartesianPose at origin
    """
    return CartesianPose(
        position=[0.0, 0.0, 0.0],
        orientation=[0.0, 0.0, 0.0, 1.0]  # Identity quaternion
    )


def create_language_instruction(text: str, source: str = "user") -> LanguageInstruction:
    """
    Create a LanguageInstruction with the specified text
    """
    return LanguageInstruction(text=text, source=source)


def create_robot_action(action_type: ActionType, values: List[float]) -> RobotAction:
    """
    Create a RobotAction with the specified type and values
    """
    return RobotAction(action_type=action_type, values=values)


if __name__ == "__main__":
    # Example usage of the data structures
    print("Testing VLA Data Structures...")
    
    # Create a joint state
    joint_state = create_default_joint_state(7)
    print(f"Created joint state: {joint_state.positions}")
    
    # Create a Cartesian pose
    pose = create_default_cartesian_pose()
    print(f"Created Cartesian pose: position={pose.position}, orientation={pose.orientation}")
    
    # Create a language instruction
    instruction = create_language_instruction("Pick up the red cube", "user")
    print(f"Created language instruction: {instruction.text}")
    
    # Create a robot action
    action = create_robot_action(ActionType.JOINT_POSITIONS, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    print(f"Created robot action: type={action.action_type}, values={action.values}")
    
    # Create a task specification
    task = TaskSpecification(
        task_type=TaskType.MANIPULATION,
        description="Pick up the red cube and place it in the box",
        language_instruction=instruction,
        target_objects=["red_cube"],
        target_locations=["box"]
    )
    print(f"Created task specification: {task.description}")
    
    # Serialize and deserialize
    task_json = DataSerializer.serialize(task)
    print(f"Serialized task to JSON: {len(task_json)} characters")
    
    restored_task = DataSerializer.deserialize(task_json, TaskSpecification)
    print(f"Deserialized task: {restored_task.description}")
    
    print("VLA Data Structures test completed.")