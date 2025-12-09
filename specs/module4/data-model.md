# Data Model: Module 4 - Vision-Language-Action Models

## Overview
This document defines the key data models and entities used in Module 4: Vision-Language-Action Models – From Voice to Physical Action. These data models provide the structure for information exchange between different components of the VLA system.

## Entity Relationships
```
[User] -(gives command)-> [LanguageInstruction]
[LanguageInstruction] -(processed by)-> [VLAPrediction]
[VLAPrediction] -(validated by)-> [SafetySystem] -(approves)-> [RobotAction]
[RobotAction] -(executed on)-> [Robot]
[Robot] -(observes)-> [VisionObservation]
[VisionObservation] -(processed by)-> [PerceptionSystem] -(updates)-> [SystemState]
```

## Core Data Models

### 1. LanguageInstruction
Represents a natural language command from the user

| Field | Type | Description |
|-------|------|-------------|
| text | string | The raw text of the instruction |
| language | string | Language code (default: "en") |
| confidence | float | Confidence in the transcription (0.0-1.0) |
| timestamp | float | Unix timestamp of when instruction was received |
| source | enum | Source of instruction (user, system, generated) |
| intent | string | Parsed intent of the command (optional) |
| entities | Map<string, string> | Parsed entities in the command (optional) |
| original_command | string | Original voice command if speech-to-text (optional) |

### 2. VisionObservation
Captures the visual state of the robot's environment

| Field | Type | Description |
|-------|------|-------------|
| image_data | bytes | Raw image data (optional, due to size) |
| depth_data | 2D array of float | Depth map values |
| camera_intrinsics | list of float | Camera intrinsic parameters [fx, fy, cx, cy] |
| camera_extrinsics | 4x4 matrix | Camera pose relative to robot base |
| objects | list of ObjectDetection | Detected objects in the scene |
| segmentation_mask | bytes | Segmentation mask for the image |
| timestamp | float | Unix timestamp of observation |
| source_camera | string | Name of the camera that captured the image |

### ObjectDetection
A detected object within the scene

| Field | Type | Description |
|-------|------|-------------|
| name | string | Object class name (e.g., "red_cube", "table") |
| confidence | float | Detection confidence (0.0-1.0) |
| bounding_box | list of int | [x, y, width, height] in pixels |
| center_3d | list of float | 3D position [x, y, z] in meters relative to camera |
| properties | Map<string, string> | Additional properties (color, size, etc.) |

### 3. VLAPrediction
Output from the Vision-Language-Action model

| Field | Type | Description |
|-------|------|-------------|
| action | list of float | Raw action values from the model |
| action_type | enum | Type of action (joint_positions, cartesian_pose, etc.) |
| confidence | float | Model confidence in the prediction |
| visual_features | list of float | Extracted visual features |
| language_features | list of float | Extracted language features |
| multimodal_features | list of float | Combined multimodal features |
| raw_output | string | Raw string output from the model |
| processed_output | list of float | Robot-appropriate processed action |
| timestamp | float | Unix timestamp of prediction |

### 4. RobotAction
Action to be executed on the robot with safety validation

| Field | Type | Description |
|-------|------|-------------|
| action_type | enum | Type of action (joint_positions, joint_velocities, etc.) |
| values | list of float | Values for the action |
| duration | float | Expected execution time in seconds (optional) |
| safety_limits | Map<string, float> | Safety parameters for execution |
| task_id | string | ID of the associated task |
| timestamp | float | Unix timestamp when action was created |
| execution_status | enum | Status: pending, in_progress, completed, failed |

### 5. SystemState
Complete state of the VLA system at a point in time

| Field | Type | Description |
|-------|------|-------------|
| robot_joint_state | JointState | Current joint positions/velocities/efforts |
| robot_pose | CartesianPose | Current robot base pose |
| latest_vision | VisionObservation | Most recent visual observation |
| latest_language | LanguageInstruction | Most recent language command |
| current_task | TaskSpecification | Currently executing task (optional) |
| task_execution_state | TaskExecutionState | State of current task execution |
| system_uptime | float | Time since system initialization |
| active_safety_events | list of SafetyEvent | Currently active safety events |
| performance_metrics | Map<string, float> | System performance metrics |
| timestamp | float | Unix timestamp of state capture |
| system_id | string | Unique identifier for the system instance |

### JointState
Robot joint state information

| Field | Type | Description |
|-------|------|-------------|
| positions | list of float | Joint positions in radians |
| velocities | list of float | Joint velocities in rad/s (optional) |
| efforts | list of float | Joint efforts in N*m (optional) |
| timestamp | float | Unix timestamp of joint state |
| joint_names | list of string | Names of joints (optional) |

### CartesianPose
3D pose in Cartesian space

| Field | Type | Description |
|-------|------|-------------|
| position | list of float | [x, y, z] position in meters |
| orientation | list of float | [x, y, z, w] quaternion orientation |
| reference_frame | string | Reference frame for the pose |
| timestamp | float | Unix timestamp of pose |

### 6. TaskSpecification
Complete specification for a robot task

| Field | Type | Description |
|-------|------|-------------|
| task_type | enum | Type of task (manipulation, navigation, etc.) |
| description | string | Human-readable description of the task |
| language_instruction | LanguageInstruction | Instruction that initiated the task |
| target_objects | list of string | Names of target objects |
| target_locations | list of string | Names of target locations |
| constraints | Map<string, any> | Task-specific constraints |
| success_criteria | list of string | Criteria for task success |
| priority | int | Priority level (1-5) |
| timeout | float | Maximum time for task completion in seconds |
| timestamp | float | Unix timestamp of task creation |
| task_id | string | Unique task identifier |

### 7. TaskExecutionState
State of task execution progress

| Field | Type | Description |
|-------|------|-------------|
| task_id | string | ID of the task being executed |
| status | enum | Current status (planning, executing, completed, failed, cancelled) |
| current_step | int | Current step in the task execution |
| total_steps | int | Total steps in the task |
| progress | float | Completion progress (0.0-1.0) |
| current_action | RobotAction | Currently executing action (optional) |
| execution_history | list of Map | History of executed actions and outcomes |
| error_message | string | Error message if task failed (optional) |
| start_time | float | Unix timestamp of task start |
| end_time | float | Unix timestamp of task end (optional) |

### 8. SafetyEvent
Record of a safety-related event

| Field | Type | Description |
|-------|------|-------------|
| timestamp | float | Unix timestamp of the event |
| level | enum | Safety level (normal, warning, emergency) |
| source | string | Component that triggered the event |
| message | string | Description of the event |
| action_taken | string | Action taken in response to the event |
| context | Map<string, any> | Additional context information |

## Action Type Enum
```
ActionType:
- JOINT_POSITIONS
- JOINT_VELOCITIES  
- JOINT_EFFORTS
- CARTESIAN_POSE
- CARTESIAN_TWIST
- GRIPPER_COMMAND
- BASE_MOTION
- COMPOSITE
```

## Task Type Enum
```
TaskType:
- MANIPULATION
- NAVIGATION
- INSPECTION
- ASSEMBLY
- HANDOVER
- STORAGE
```

## API Contracts

### VLA Inference Service
- **Request**: {vision_observation: VisionObservation, language_instruction: LanguageInstruction}
- **Response**: {vla_prediction: VLAPrediction, success: boolean, message: string}

### Action Validation Service  
- **Request**: {robot_action: RobotAction}
- **Response**: {is_safe: boolean, violations: list of string, message: string}

### Task Execution Service
- **Request**: {task_specification: TaskSpecification}
- **Response**: {task_id: string, execution_started: boolean, message: string}

### System State Service
- **Request**: {}
- **Response**: {system_state: SystemState}