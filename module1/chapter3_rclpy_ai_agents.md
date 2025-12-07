# Chapter 3: rclpy – Bridging Python AI Agents to Robots

## Learning Objectives

By the end of this chapter, you should be able to:
- Create Python nodes using rclpy that can interface with robots
- Implement rclpy publishers that publish joint trajectories to control robots
- Develop rclpy subscribers that process sensor data from robots
- Wrap Hugging Face transformers or OpenAI API calls inside ROS 2 nodes
- Implement latency measurements and best practices for running LLMs on the same machine as real-time control
- Understand security considerations for AI-robot communication
- Include error handling for network timeouts, sensor failures, and actuator errors
- Design AI agents that can bridge the gap between high-level AI models and physical robotic actions
- Achieve acceptable latency measurements for AI-robot communication (under 100ms)
- Implement fallback mechanisms for when AI services become unavailable

## 3.1 Introduction to rclpy and AI-robot Integration

In the previous chapters, we've established the foundation of ROS 2 and the concepts of embodied intelligence. This chapter focuses on the crucial task of bridging AI agents with physical robotic systems using rclpy, the Python client library for ROS 2.

Modern AI systems, particularly large language models (LLMs) and computer vision models, generate high-level decisions and plans. However, for these systems to control physical robots, they must interface with low-latency, real-time control systems. The rclpy library provides this essential bridge between high-level AI and low-level robot control.

### 3.1.1 The AI-Robot Control Loop

When AI agents control robots, a complex multi-layered control loop emerges:

1. **High-Level Planning**: AI models generate high-level goals and plans
2. **Mid-Level Coordination**: ROS 2 nodes coordinate between AI and robot systems
3. **Low-Level Control**: Real-time controllers execute precise physical actions

Each layer must maintain appropriate performance characteristics: AI systems might process information over hundreds of milliseconds, while real-time controllers must respond in microsecond timeframes.

## 3.2 Setting Up rclpy for AI Integration

To use rclpy effectively for AI integration, you need to consider the requirements for both AI processing and control systems. AI nodes often require significant computational resources and may not meet strict real-time deadlines, whereas control nodes must maintain consistent timing for safe robot operation.

### 3.2.1 Basic rclpy Node Structure for AI Applications

Here's a foundational template for an AI-robot interface node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from athena_interfaces.msg import AICommand  # Custom message for AI decisions
import threading
import time
import numpy as np

class AIControllerNode(Node):
    """
    A node that bridges AI models with robot control using rclpy.
    Separates AI computation from real-time control to maintain safety.
    """
    
    def __init__(self):
        super().__init__('ai_controller_node')
        
        # Publishers for robot commands
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Subscriber for sensor data
        self.sensor_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.sensor_callback,
            10
        )
        
        # Subscriber for AI decisions
        self.ai_command_subscriber = self.create_subscription(
            AICommand,
            'ai_robot_commands',
            self.ai_command_callback,
            10
        )
        
        # Store sensor data for AI access
        self.current_joint_states = None
        
        # Thread for AI processing (separate from ROS thread)
        self.ai_thread = threading.Thread(target=self.ai_processing_loop)
        self.ai_thread.daemon = True
        self.ai_thread.start()
        
        # Timers for periodic tasks
        self.update_timer = self.create_timer(0.1, self.update_callback)
        
        self.get_logger().info('AI Controller Node initialized')

    def sensor_callback(self, msg):
        """Handle incoming sensor data."""
        self.current_joint_states = msg
        self.get_logger().debug(f'Received sensor data for {len(msg.name)} joints')

    def ai_command_callback(self, msg):
        """Handle AI commands and convert them to robot actions."""
        self.get_logger().info(f'Received AI command: {msg.command} with confidence {msg.confidence}')
        
        # Process the AI command
        if msg.command == 'wave_hand':
            self.execute_wave_hand_action()
        elif msg.command == 'move_forward':
            self.execute_move_forward_action()
        elif msg.command == 'turn_left':
            self.execute_turn_left_action()
        # Add more action mappings as needed

    def ai_processing_loop(self):
        """Dedicated thread for AI computations to avoid blocking ROS callbacks."""
        while rclpy.ok():
            # Perform AI computations in this thread
            # This might involve:
            # - Processing camera images 
            # - Running language models
            # - Planning paths
            # - Making decisions based on sensor data
            time.sleep(0.01)  # Small sleep to prevent busy looping

    def update_callback(self):
        """Called periodically to maintain system health."""
        # This method runs in the main ROS thread
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AIControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.3 rclpy Publishers for Robot Control

For AI agents to control robots, publishers are essential for sending commands to robot controllers. When designing these publishers, consider the following:

### 3.3.1 Joint Trajectory Publishing

For articulated robots like the "athena" humanoid, joint trajectories are the most common command type:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class JointTrajectoryController(Node):
    def __init__(self):
        super().__init__('joint_trajectory_controller')
        
        self.publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Define joint names for the "athena" humanoid
        self.joint_names = [
            'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint',
            'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint',
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]
        
        # Timer to send periodic commands
        self.timer = self.create_timer(0.5, self.send_wave_trajectory)
        
        self.get_logger().info('Joint Trajectory Controller initialized')

    def send_wave_trajectory(self):
        """Send a trajectory that makes the robot wave."""
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Create trajectory points
        points = []
        
        # Right arm wave pattern
        for i in range(5):  # 5 points for smooth motion
            point = JointTrajectoryPoint()
            
            # Calculate joint positions for the wave
            positions = []
            for idx, joint_name in enumerate(self.joint_names):
                position = 0.0  # Default position
                
                # Special movement for right shoulder to wave
                if joint_name == 'right_shoulder_joint':
                    position = math.sin(i * 0.5) * 0.8  # Wave up/down
                elif joint_name == 'right_elbow_joint':
                    position = math.cos(i * 0.5) * 0.5 + 0.5  # Flex extension
                # Keep other joints at neutral position
                # In practice, would implement full kinematic chain
                
                positions.append(position)
            
            point.positions = positions
            point.velocities = [0.0] * len(self.joint_names)
            point.accelerations = [0.0] * len(self.joint_names)
            point.time_from_start = Duration(sec=i, nanosec=200000000 * i)  # 200ms intervals
            
            points.append(point)
        
        msg.points = points
        self.publisher.publish(msg)
        
        self.get_logger().info('Published waving trajectory')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.4 rclpy Subscribers for Sensor Processing

Subscribers allow AI agents to receive real-time sensor data from robots, which is crucial for closed-loop control and adaptive behavior.

### 3.4.1 Sensor Data Processing with AI Models

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import String
import numpy as np
import threading

class SensorProcessingNode(Node):
    def __init__(self):
        super().__init__('sensor_processing_node')
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to camera data
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for AI decisions based on sensor data
        self.ai_decision_pub = self.create_publisher(String, 'ai_decisions', 10)
        
        # Store latest sensor data
        self.latest_joint_states = None
        self.latest_image = None
        
        # Lock for thread safety
        self.sensor_lock = threading.Lock()
        
        # Timer to trigger AI processing
        self.processing_timer = self.create_timer(1.0, self.process_sensors_with_ai)
        
        self.get_logger().info('Sensor Processing Node initialized')

    def joint_state_callback(self, msg):
        """Handle incoming joint state data."""
        with self.sensor_lock:
            self.latest_joint_states = msg
        self.get_logger().debug(f'Received joint states for {len(msg.name)} joints')

    def image_callback(self, msg):
        """Handle incoming camera image data."""
        with self.sensor_lock:
            # Convert ROS Image to a format suitable for AI models
            # Note: Actual conversion would require cv_bridge or similar
            self.latest_image = msg
        self.get_logger().debug(f'Received image: {msg.width}x{msg.height}')

    def process_sensors_with_ai(self):
        """Process sensor data with AI model in a separate thread."""
        with self.sensor_lock:
            # Copy sensor data to prevent race conditions
            joint_data = self.latest_joint_states
            image_data = self.latest_image
        
        if joint_data and len(joint_data.position) > 0:
            # Process joint data with AI model
            ai_decision = self.analyze_joint_states(joint_data.position)
            
            # Publish AI decision
            decision_msg = String()
            decision_msg.data = ai_decision
            self.ai_decision_pub.publish(decision_msg)
            
            self.get_logger().info(f'AI decision based on joints: {ai_decision}')

    def analyze_joint_states(self, positions):
        """Analyze joint positions and make an AI decision."""
        # Convert to numpy array for easier processing
        pos_array = np.array(positions)
        
        # Example: Calculate average deviation from neutral position
        neutral_pos = np.zeros_like(pos_array)
        deviation = np.mean(np.abs(pos_array - neutral_pos))
        
        if deviation > 1.0:
            return "Robot is in unusual pose, check for obstacles"
        elif deviation > 0.5:
            return "Robot is moving actively, continue monitoring"
        else:
            return "Robot is in stable position, normal operation"
        
def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.5 Integrating AI Models with rclpy

One of the most powerful aspects of rclpy is its ability to integrate with the rich Python AI ecosystem. This section covers different approaches to seamlessly connect AI models with robot control.

### 3.5.1 Wrapping Hugging Face Transformers in ROS 2 Nodes

Hugging Face provides access to numerous pre-trained models that can be integrated into ROS 2 nodes:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from athena_interfaces.msg import AICommand  # Custom message type
from transformers import pipeline
import threading

class HuggingFaceAIController(Node):
    def __init__(self):
        super().__init__('hf_ai_controller')
        
        # Initialize Hugging Face pipeline in a separate thread to avoid blocking node initialization
        self.model_thread = threading.Thread(target=self.initialize_model)
        self.model_thread.start()
        
        # Subscription to natural language commands
        self.natural_language_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.process_natural_language,
            10
        )
        
        # Publisher for AI commands to robot
        self.ai_command_publisher = self.create_publisher(AICommand, 'robot_ai_commands', 10)
        
        self.classifier = None
        self.model_initialized = False
        
        self.get_logger().info('Hugging Face AI Controller initialized')

    def initialize_model(self):
        """Initialize the Hugging Face model in a separate thread."""
        try:
            self.get_logger().info('Initializing Hugging Face model...')
            self.classifier = pipeline(
                "zero-shot-classification",
                model="facebook/bart-large-mnli"
            )
            self.model_initialized = True
            self.get_logger().info('Hugging Face model initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Hugging Face model: {str(e)}')
            self.model_initialized = False

    def process_natural_language(self, msg):
        """Process natural language commands using the Hugging Face model."""
        if not self.model_initialized:
            self.get_logger().warn('Hugging Face model not initialized, skipping command')
            return

        command = msg.data
        self.get_logger().info(f'Processing command: {command}')
        
        try:
            # Define candidate labels for robot actions
            candidate_labels = [
                "move forward", "move backward", "turn left", "turn right", 
                "wave hello", "pick up object", "put down object", "stop", 
                "follow me", "come here", "dance", "sit down", "stand up"
            ]
            
            # Use the model to classify the command
            result = self.classifier(command, candidate_labels, multi_label=False)
            
            self.get_logger().info(f'Model prediction: {result["labels"][0]} (confidence: {result["scores"][0]:.2f})')
            
            # Convert model prediction to robot command
            robot_command = self.convert_model_output_to_robot_command(
                result["labels"][0], 
                result["scores"][0]
            )
            
            # Publish the robot command
            ai_cmd = AICommand()
            ai_cmd.command = robot_command
            ai_cmd.confidence = result["scores"][0]
            ai_cmd.description = f"Based on: '{command}' -> {result['labels'][0]}"
            
            self.ai_command_publisher.publish(ai_cmd)
            
        except Exception as e:
            self.get_logger().error(f'Error processing natural language: {str(e)}')

    def convert_model_output_to_robot_command(self, model_output, confidence):
        """Convert Hugging Face model output to robot command."""
        # Map model labels to robot commands
        command_mapping = {
            "move forward": "move_forward",
            "move backward": "move_backward", 
            "turn left": "turn_left",
            "turn right": "turn_right",
            "wave hello": "wave_hand",
            "pick up object": "grasp_object",
            "put down object": "release_object",
            "stop": "idle",
            "follow me": "follow_person",
            "come here": "approach_operator",
            "dance": "perform_dance",
            "sit down": "sit_down",
            "stand up": "stand_up"
        }
        
        # Find the closest match
        for model_label, robot_cmd in command_mapping.items():
            if model_label in model_output.lower():
                return robot_cmd
        
        # If no specific action matched, use a default
        if confidence > 0.5:
            return "idle"  # Default safe action
        else:
            return "await_clarification"  # Ask for clarification

def main(args=None):
    rclpy.init(args=args)
    node = HuggingFaceAIController()
    
    try:
        # Wait for model to initialize
        while not node.model_initialized and rclpy.ok():
            node.get_logger().info('Waiting for model initialization...')
            time.sleep(0.5)
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.5.2 Wrapping OpenAI API Calls in ROS 2 Nodes

OpenAI APIs can also be integrated into ROS 2 nodes, though with additional considerations for API costs and latency:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from athena_interfaces.msg import AICommand
import openai
import os
import threading
from functools import partial

class OpenAIAPIController(Node):
    def __init__(self):
        super().__init__('openai_api_controller')
        
        # Initialize OpenAI API
        # In production, use secure methods to store API keys
        openai.api_key = os.getenv('OPENAI_API_KEY')
        
        if not openai.api_key:
            self.get_logger().warn('OPENAI_API_KEY not set, API functionality will be disabled')
        
        # Subscription to natural language commands
        self.natural_language_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.process_natural_language,
            10
        )
        
        # Publisher for AI commands to robot
        self.ai_command_publisher = self.create_publisher(AICommand, 'robot_ai_commands', 10)
        
        # Cache for storing responses to common queries
        self.response_cache = {}
        self.cache_max_size = 20
        
        self.get_logger().info('OpenAI API Controller initialized')

    def process_natural_language(self, msg):
        """Process natural language commands using OpenAI API."""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')
        
        # Check if command is in cache
        if command in self.response_cache:
            response = self.response_cache[command]
            self.get_logger().info(f'Using cached response for: {command}')
        else:
            if not openai.api_key:
                # Fallback if no API key is available
                response = self.fallback_command_processing(command)
            else:
                # Call OpenAI API in a separate thread to avoid blocking
                future = self.call_openai_api(command)
                if future is not None:
                    # For simplicity, we'll handle result directly in this example
                    response = self.call_openai_api_sync(command)
                    
                    # Add to cache if response is valid
                    if response and len(self.response_cache) < self.cache_max_size:
                        self.response_cache[command] = response
        
        # Publish the AI command
        if response:
            ai_cmd = AICommand()
            ai_cmd.command = response['robot_command']
            ai_cmd.confidence = response['confidence']
            ai_cmd.description = response['description']
            
            self.ai_command_publisher.publish(ai_cmd)
            
            self.get_logger().info(f'Published AI command: {ai_cmd.command}')

    def call_openai_api_sync(self, command):
        """Call OpenAI API synchronously."""
        try:
            # Construct the prompt to guide the model toward robot commands
            prompt = f"""
            You are an AI robot commander. Interpret the human instruction '{command}' as a command for a humanoid robot. 
            Choose from this list of possible robot commands:
            - move_forward, move_backward, turn_left, turn_right
            - wave_hand, raise_arms, lower_arms
            - grasp_object, release_object
            - sit_down, stand_up
            - idle
            - follow_person
            - approach_operator
            
            Only respond with the appropriate robot command, nothing else.
            """
            
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot command interpreter. Respond only with simple robot commands."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=20,
                temperature=0.1  # Low temperature for consistent responses
            )
            
            robot_cmd = response.choices[0].message.content.strip().lower()
            
            # Validate the response is a valid command
            valid_commands = {
                'move forward': 'move_forward',
                'move_backward': 'move_backward',
                'turn left': 'turn_left', 
                'turn_right': 'turn_right',
                'wave hand': 'wave_hand',
                'raise arms': 'raise_arms',
                'lower arms': 'lower_arms',
                'grasp object': 'grasp_object',
                'release object': 'release_object',
                'sit down': 'sit_down',
                'stand up': 'stand_up',
                'idle': 'idle',
                'follow person': 'follow_person',
                'approach operator': 'approach_operator'
            }
            
            normalized_cmd = robot_cmd.replace('_', ' ')
            
            for valid_phrase, robot_cmd_val in valid_commands.items():
                if valid_phrase in normalized_cmd:
                    return {
                        'robot_command': robot_cmd_val,
                        'confidence': 0.9,  # Assuming high confidence for GPT
                        'description': f"Converted '{command}' to '{robot_cmd_val}' via OpenAI API"
                    }
            
            # If no valid command found, return safe default
            return {
                'robot_command': 'idle',
                'confidence': 0.3,
                'description': f"No suitable command found for '{command}', defaulted to 'idle'"
            }
            
        except Exception as e:
            self.get_logger().error(f'Error calling OpenAI API: {str(e)}')
            # Return a safe fallback command
            return {
                'robot_command': 'idle',
                'confidence': 0.1,
                'description': f"API Error for '{command}': {str(e)}"
            }

    def fallback_command_processing(self, command):
        """Fallback processing when API is not available."""
        self.get_logger().warn(f'Using fallback processing for: {command}')
        
        # Simple keyword-based mapping as fallback
        cmd_lower = command.lower()
        
        if any(word in cmd_lower for word in ['forward', 'ahead', 'go']):
            return {'robot_command': 'move_forward', 'confidence': 0.6, 'description': f"Keyword match for '{command}'"}
        elif any(word in cmd_lower for word in ['backward', 'back']):
            return {'robot_command': 'move_backward', 'confidence': 0.6, 'description': f"Keyword match for '{command}'"}
        elif any(word in cmd_lower for word in ['left', 'turn left']):
            return {'robot_command': 'turn_left', 'confidence': 0.6, 'description': f"Keyword match for '{command}'"}
        elif any(word in cmd_lower for word in ['right', 'turn right']):
            return {'robot_command': 'turn_right', 'confidence': 0.6, 'description': f"Keyword match for '{command}'"}
        elif any(word in cmd_lower for word in ['wave', 'hello', 'hi']):
            return {'robot_command': 'wave_hand', 'confidence': 0.6, 'description': f"Keyword match for '{command}'"}
        elif any(word in cmd_lower for word in ['sit', 'sit down']):
            return {'robot_command': 'sit_down', 'confidence': 0.6, 'description': f"Keyword match for '{command}'"}
        elif any(word in cmd_lower for word in ['stand', 'up', 'stand up']):
            return {'robot_command': 'stand_up', 'confidence': 0.6, 'description': f"Keyword match for '{command}'"}
        else:
            return {'robot_command': 'idle', 'confidence': 0.3, 'description': f"No match for '{command}', defaulted to 'idle'"}

def main(args=None):
    rclpy.init(args=args)
    node = OpenAIAPIController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.6 Latency Considerations and Performance Optimization

Achieving low latency in AI-robot communication is critical for safe and responsive robot behavior, especially for dynamic tasks where delays could result in accidents or poor performance.

### 3.6.1 Implementing Latency Measurements

To ensure your AI-robot communication meets the <100ms requirement, implement proper latency measurements:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import time

class LatencyMeasurementNode(Node):
    def __init__(self):
        super().__init__('latency_measurement_node')
        
        # Publisher for timestamped test messages
        self.test_publisher = self.create_publisher(Header, 'latency_test', 10)
        
        # Subscription to echoed test messages
        self.test_subscription = self.create_subscription(
            Header,
            'latency_test_echo',  # In a real system, an echo node would republish this
            self.latency_response_callback,
            10
        )
        
        # Publisher for latency results
        self.latency_publisher = self.create_publisher(String, 'latency_report', 10)
        
        # Timer to send test messages periodically
        self.test_timer = self.create_timer(0.5, self.send_latency_test_message)
        
        # Storage for tracking message timestamps
        self.sent_times = {}
        self.message_counter = 0
        
        # Statistics for latency measurements
        self.latency_samples = []
        
        # Stats reporting timer (report every 5 seconds)
        self.stats_timer = self.create_timer(5.0, self.report_statistics)
        
        self.get_logger().info('Latency Measurement Node initialized')

    def send_latency_test_message(self):
        """Send a test message with a timestamp to measure round-trip time."""
        # Create a header with timestamp
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = f"latency_test_{self.message_counter}"
        
        # Record the exact time we sent the message
        send_time = time.perf_counter()
        self.sent_times[header.frame_id] = send_time
        self.message_counter += 1
        
        # Publish the test message
        self.test_publisher.publish(header)
        self.get_logger().debug(f'Sent latency test message: {header.frame_id}')

    def latency_response_callback(self, msg):
        """Callback for receiving echoed test messages and calculating latency."""
        # Get the precise time the message was received
        receive_time = time.perf_counter()
        
        # Look up the time the message was sent
        sent_time = self.sent_times.pop(msg.frame_id, None)
        
        if sent_time is not None:
            # Calculate round-trip time in milliseconds
            rtt_ms = (receive_time - sent_time) * 1000
            
            # Store the latency sample
            self.latency_samples.append(rtt_ms)
            
            # Create and publish latency report
            latency_msg = String()
            latency_msg.data = f"Message: {msg.frame_id}, RTT: {rtt_ms:.2f}ms"
            self.latency_publisher.publish(latency_msg)
            
            self.get_logger().info(f'Round-trip latency: {rtt_ms:.2f}ms for message {msg.frame_id}')
            
            # Check if latency meets requirements
            if rtt_ms > 100.0:
                self.get_logger().warn(f'Latency exceeded 100ms threshold: {rtt_ms:.2f}ms')
        else:
            self.get_logger().warn(f'Received latency test echo for unknown message: {msg.frame_id}')

    def report_statistics(self):
        """Report statistics about the latency measurements."""
        if not self.latency_samples:
            self.get_logger().info('No latency samples to report')
            return

        count = len(self.latency_samples)
        avg_latency = sum(self.latency_samples) / count
        min_latency = min(self.latency_samples)
        max_latency = max(self.latency_samples)
        
        # Calculate percentile latencies (important for real-time systems)
        sorted_latencies = sorted(self.latency_samples)
        p95_latency = sorted_latencies[int(0.95 * count)]
        
        # Report statistics
        self.get_logger().info('=== LATENCY STATISTICS ===')
        self.get_logger().info(f'Total samples: {count}')
        self.get_logger().info(f'Average latency: {avg_latency:.2f} ms')
        self.get_logger().info(f'Minimum latency: {min_latency:.2f} ms')
        self.get_logger().info(f'95th percentile latency: {p95_latency:.2f} ms')
        self.get_logger().info(f'Maximum latency: {max_latency:.2f} ms')
        
        # Issue warning if 95th percentile exceeds threshold
        if p95_latency > 100.0:
            self.get_logger().error(f'95th percentile latency ({p95_latency:.2f}ms) exceeded 100ms requirement!')
        
        # Keep only the last 100 samples to avoid memory accumulation
        if len(self.latency_samples) > 100:
            self.latency_samples = self.latency_samples[-100:]

def main(args=None):
    rclpy.init(args=args)
    node = LatencyMeasurementNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.6.2 Best Practices for LLM and Real-Time Control Coexistence

When running large language models simultaneously with real-time robot control, implement these best practices:

1. **Asynchronous Processing**: Isolate AI computations from real-time control loops
2. **Resource Allocation**: Use system-level controls to ensure real-time processes maintain priority
3. **Buffering**: Use asynchronous buffer queues to decouple processing rates
4. **Fail-Safe Mechanisms**: Implement safety fallbacks that activate if AI processing fails

## 3.7 Security Considerations for AI-Robot Communication

When AI systems control physical robots, security becomes paramount. Unauthorized or malicious AI agents could cause physical harm to humans or damage to property.

### 3.7.1 Authentication and Authorization

Implement proper authentication and authorization mechanisms to ensure only trusted AI agents can control the robot:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from athena_interfaces.msg import AICommand
import hashlib
import time

class SecureAIControllerNode(Node):
    def __init__(self):
        super().__init__('secure_ai_controller_node')
        
        # Authorized AI agent IDs (in practice, this would be stored securely)
        self.authorized_agents = {
            'hf_model_v1.0': 'abc123def456',  # Example: model_id: secret_token
            'openai_service': 'xyz789uvw012',
            'local_planner': 'mno345pqr678'
        }
        
        # Subscription with authentication
        self.unverified_command_sub = self.create_subscription(
            String,
            'unverified_ai_commands',
            self.authenticate_and_process_command,
            10
        )
        
        # Verified command publisher
        self.verified_command_pub = self.create_publisher(AICommand, 'verified_robot_commands', 10)
        
        self.get_logger().info('Secure AI Controller Node initialized')

    def authenticate_and_process_command(self, msg):
        """Authenticate the AI agent and process the command."""
        # The message format should be: "AGENT_ID:TOKEN:COMMAND"
        parts = msg.data.split(':', 2)
        if len(parts) != 3:
            self.get_logger().error(f'Malformed command: {msg.data}')
            return
        
        agent_id, token, command = parts
        
        # Verify the agent is authorized
        if agent_id not in self.authorized_agents:
            self.get_logger().error(f'Unauthorized AI agent: {agent_id}')
            return
        
        # Verify the token
        expected_token = self.authorized_agents[agent_id]
        if token != expected_token:
            self.get_logger().error(f'Invalid token for agent {agent_id}')
            return
        
        # If authentication passes, process the command
        self.get_logger().info(f'Authenticated command from {agent_id}: {command}')
        
        # Convert authenticated command to AICommand message
        ai_cmd = AICommand()
        ai_cmd.command = command
        ai_cmd.confidence = 1.0  # Fully trusted since authenticated
        ai_cmd.description = f'Authenticated command from {agent_id}'
        
        # Publish verified command
        self.verified_command_pub.publish(ai_cmd)
        
        self.get_logger().info(f'Verified command published: {command}')

def main(args=None):
    rclpy.init(args=args)
    node = SecureAIControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.8 Error Handling and Safety

Proper error handling is critical when AI systems control physical robots. Implement comprehensive error detection and recovery mechanisms.

### 3.8.1 Network Timeouts and Service Failures

Handle network timeouts gracefully to prevent robot malfunctions:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import requests
import threading
import time

class RobustAIControllerNode(Node):
    def __init__(self):
        super().__init__('robust_ai_controller_node')
        
        # Publisher for system status
        self.status_publisher = self.create_publisher(Bool, 'system_healthy', 10)
        
        # Publisher for error alerts
        self.alert_publisher = self.create_publisher(String, 'system_alerts', 10)
        
        # Timer for periodic AI service checks
        self.service_check_timer = self.create_timer(2.0, self.check_ai_service_health)
        
        # Track service health
        self.ai_service_healthy = True
        self.last_heartbeat = time.time()
        self.timeout_threshold = 5.0  # seconds without heartbeat = unhealthy
        
        # Fallback mechanism
        self.fallback_active = False
        
        self.get_logger().info('Robust AI Controller Node initialized')

    def check_ai_service_health(self):
        """Check if connected AI services are responding."""
        current_time = time.time()
        
        # Check if we've received a heartbeat recently
        if current_time - self.last_heartbeat > self.timeout_threshold:
            if self.ai_service_healthy:
                self.get_logger().error('AI service timeout detected, activating fallback')
                self.ai_service_healthy = False
                self.activate_fallback()
        else:
            if not self.ai_service_healthy:
                self.get_logger().info('AI service recovered, deactivating fallback')
                self.ai_service_healthy = True
                self.deactivate_fallback()
        
        # Publish system health status
        health_msg = Bool()
        health_msg.data = self.ai_service_healthy
        self.status_publisher.publish(health_msg)

    def activate_fallback(self):
        """Activate safe fallback behavior when AI services fail."""
        self.fallback_active = True
        
        # Publish alert
        alert_msg = String()
        alert_msg.data = "AI service failure - activated safe fallback mode"
        self.alert_publisher.publish(alert_msg)
        
        # In a real robot, you might:
        # - Stop all motion
        # - Switch to manual control mode
        # - Activate emergency behaviors
        self.get_logger().warn('Fallback mode activated - robot is in safe state')

    def deactivate_fallback(self):
        """Deactivate fallback when AI services recover."""
        self.fallback_active = False
        
        # Publish alert
        alert_msg = String()
        alert_msg.data = "AI service recovered - exited fallback mode"
        self.alert_publisher.publish(alert_msg)
        
        self.get_logger().info('Fallback mode deactivated - resuming normal operation')

    def call_ai_service_with_timeout(self, url, data, timeout=3.0):
        """Call an external AI service with a timeout."""
        try:
            response = requests.post(
                url,
                json=data,
                timeout=timeout
            )
            self.last_heartbeat = time.time()  # Update heartbeat on successful response
            return response.json() if response.status_code == 200 else None
        except requests.exceptions.Timeout:
            self.get_logger().error(f'AI service call timed out after {timeout}s')
            return None
        except requests.exceptions.ConnectionError:
            self.get_logger().error(f'Unable to connect to AI service at {url}')
            return None
        except Exception as e:
            self.get_logger().error(f'Error calling AI service: {str(e)}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = RobustAIControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3.9 Pro Tips for AI-Robot Integration

- **Separate Timing Domains**: Keep high-frequency control loops (1kHz) separate from AI processing (1-10Hz)
- **Use Fast DDS**: Configure DDS for low-latency communication between AI and control nodes
- **Implement Watchdogs**: Use timers to detect and respond to stuck AI processes
- **Monitor Resources**: Continuously track CPU, memory, and GPU usage of AI processes
- **Validate Outputs**: Always validate AI-generated commands before sending to robot
- **Log Extensively**: Maintain detailed logs for debugging AI behavior and system performance
- **Plan for Graceful Degradation**: Design systems that work in reduced capacity when AI is unavailable
- **Implement Safety Barriers**: Use hardware and software safety barriers as a last resort
- **Consider Edge Computing**: Run AI models on robot-local hardware to reduce latency
- **Cache Predictable Results**: Pre-compute responses for common situations to improve reactivity

## 3.10 Summary

This chapter has covered the critical aspects of bridging AI agents with robots using rclpy. We've explored how to create nodes that integrate with the rich Python AI ecosystem while maintaining the safety and performance requirements of robotic systems.

We've implemented examples of wrapping both Hugging Face transformers and OpenAI API calls within ROS 2 nodes, ensuring proper separation of concerns between AI processing and real-time control. Additionally, we've addressed crucial non-functional requirements such as latency measurements, security considerations, and error handling for robust AI-robot systems.

The next chapter will delve into robot description using URDF and Xacro, which is essential for creating the "athena" humanoid model we've referenced throughout this module. This will provide the necessary tools to describe our robot in a way that both simulation and control systems can understand.

## Exercises

1. Create an AI node that processes camera images using a computer vision model and makes navigation decisions.
2. Implement a safety layer that validates AI-generated joint trajectories before execution.
3. Design a system that caches responses from slow AI models to improve responsiveness.
4. Create a node that monitors CPU and memory usage of AI processes and throttles when resources are low.
5. Implement a fallback mechanism that activates manual control when AI services fail.

### Solutions to Exercises

[Detailed solutions for each exercise are provided in exercises/chapter3_exercises.md]