---
sidebar_position: 4
title: Chapter 3 - rclpy and AI Agents
---

# Chapter 3: rclpy Ã¢â‚¬â€œ Bridging Python AI Agents to Robots

## Learning Objectives

By the end of this chapter, you should be able to:
- Create Python nodes using rclpy that can interface with robots
- Implement rclpy publishers that publish joint trajectories to control robots
- Develop rclpy subscribers that process sensor data from robots
- Wrap Hugging Face transformers or OpenAI API calls inside ROS 2 nodes
- Implement latency measurements and understand best practices for running LLMs alongside real-time control
- Include appropriate security considerations for AI-robot communication
- Implement error handling for network timeouts, sensor failures, and actuator errors
- Design AI agents that can bridge the gap between high-level AI models and physical robotic actions

## 3.1 Introduction to rclpy

rclpy is the Python client library for ROS 2 (Robot Operating System 2). It provides a Python API to interact with the ROS 2 middleware, allowing Python programmers to create ROS 2 nodes, communicate with other ROS 2 nodes, and access the various features of ROS 2.

The library provides access to underlying ROS concepts such as:
- Nodes
- Parameters
- Topics (Publishers and Subscribers)
- Services (Clients and Servers)
- Actions (Clients and Servers)

rclpy is built on top of rcl (the C-based client library) and provides Pythonic interfaces that make it easier to develop ROS 2 applications in Python.

### 3.1.1 Why Use Python and rclpy for AI Integration?

Python dominates the AI and machine learning ecosystem with excellent libraries for:
- Machine learning: scikit-learn, PyTorch, TensorFlow
- Natural language processing: spaCy, NLTK, Transformers
- Computer vision: OpenCV, PIL, torchvision
- Scientific computing: NumPy, SciPy, Pandas

By using rclpy, we can seamlessly integrate these powerful AI libraries with ROS 2, enabling sophisticated robotics applications where AI models can directly control robotic behavior.

## 3.2 Setting Up rclpy Nodes

To create a basic rclpy node, you need to:
1. Import the required modules
2. Create a class that inherits from rclpy.node.Node
3. Initialize the node in the constructor
4. Define the node's behavior

Here's a minimal rclpy node template:

```python
import rclpy
from rclpy.node import Node

class BasicAINode(Node):
    def __init__(self):
        super().__init__('basic_ai_node')
        self.get_logger().info('Basic AI node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = BasicAINode()
    
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

Publishers in ROS 2 allow nodes to send messages to topics, which other nodes can subscribe to. For AI agents controlling robots, publishers are used to send commands such as joint trajectories, velocity commands, or high-level goals.

### 3.3.1 Creating a Joint Trajectory Publisher

To send commands to a robot's joints, we typically use the trajectory_msgs/JointTrajectory message. Here's an example of an AI agent that publishes joint trajectories:

```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, 'joint_trajectory', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Joint names for the "athena" humanoid robot
        self.joint_names = [
            'left_shoulder_joint', 'left_elbow_joint', 'left_wrist_joint',
            'right_shoulder_joint', 'right_elbow_joint', 'right_wrist_joint',
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Create trajectory points
        point = JointTrajectoryPoint()
        
        # Calculate positions based on some AI model output
        # This is a simple example - in real applications, this would come from an AI model
        positions = []
        for i in range(len(self.joint_names)):
            # Simulate AI decision for joint position
            position = np.sin(rclpy.clock.Clock().now().nanoseconds / 1e9 + i) * 0.5
            positions.append(position)
        
        point.positions = positions
        point.velocities = [0.0] * len(self.joint_names)
        point.accelerations = [0.0] * len(self.joint_names)
        
        # Set timing information
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5 seconds
        
        msg.points = [point]
        self.publisher.publish(msg)
        self.get_logger().info(f'Published trajectory for {len(positions)} joints')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()
    
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

## 3.4 rclpy Subscribers for Sensor Data Processing

Subscribers allow nodes to receive messages from topics. For AI agents, subscribers are used to receive sensor data from robots that can be processed by AI models.

### 3.4.1 Creating a Sensor Data Subscriber

Here's an example of an AI agent that subscribes to sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String

class SensorDataProcessor(Node):
    def __init__(self):
        super().__init__('sensor_data_processor')
        
        # Subscribe to joint states from the robot
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # Publisher for AI-generated commands
        self.command_publisher = self.create_publisher(JointTrajectory, 'ai_generated_commands', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')
        
        # Process the sensor data with your AI model
        ai_output = self.process_sensor_data(msg.position)
        
        # Convert AI output to robot command
        trajectory_msg = self.create_trajectory_command(ai_output)
        self.command_publisher.publish(trajectory_msg)

    def process_sensor_data(self, joint_positions):
        """
        Process joint positions through an AI model.
        This is a placeholder - in real implementation, this would use actual AI models.
        """
        # Simulate AI processing
        # In a real application, this might:
        # - Apply a machine learning model to the sensor data
        # - Generate responses to perceived environmental conditions
        # - Plan high-level actions based on observations
        processed_output = [pos * 0.9 for pos in joint_positions]  # Example transformation
        return processed_output

    def create_trajectory_command(self, ai_output):
        """
        Convert AI model output to JointTrajectory message for robot control.
        """
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        
        # This would need actual joint names, which we'll get from somewhere
        msg = JointTrajectory()
        msg.joint_names = ['joint1', 'joint2', 'joint3']  # Placeholder
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = ai_output[:len(msg.joint_names)]  # Match number of joints
        point.velocities = [0.0] * len(point.positions)
        point.accelerations = [0.0] * len(point.positions)
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5 seconds
        
        msg.points = [point]
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = SensorDataProcessor()
    
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

One of the powerful aspects of rclpy is its ability to integrate AI models with robot control. This section covers different approaches to wrapping AI models in ROS 2 nodes.

### 3.5.1 Wrapping Hugging Face Transformers

Here's an example of how to wrap a Hugging Face transformer in a ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from athena_interfaces.msg import AICommand  # Custom message type
from transformers import pipeline  # You'll need to install transformers

class HFTransformerNode(Node):
    def __init__(self):
        super().__init__('hf_transformer_node')
        
        # Initialize the Hugging Face pipeline
        # For this example, we'll use a simple sentiment analysis model
        self.classifier = pipeline(
            "sentiment-analysis",
            model="cardiffnlp/twitter-roberta-base-sentiment-latest"
        )
        
        # Subscribe to text commands
        self.text_subscription = self.create_subscription(
            String,
            'natural_language_command',
            self.text_callback,
            10
        )
        
        # Publisher for AI-generated robot commands
        self.command_publisher = self.create_publisher(AICommand, 'ai_robot_command', 10)

    def text_callback(self, msg):
        self.get_logger().info(f'Received text: {msg.data}')
        
        # Process with the Hugging Face model
        result = self.classifier(msg.data)
        
        # Convert sentiment analysis result to robot action
        robot_cmd = self.sentiment_to_robot_action(result[0])
        
        # Create and publish the AI command
        ai_cmd_msg = AICommand()
        ai_cmd_msg.command = robot_cmd['command']
        ai_cmd_msg.confidence = robot_cmd['confidence']
        ai_cmd_msg.description = f"Based on: {result[0]['label']} with score {result[0]['score']}"
        
        self.command_publisher.publish(ai_cmd_msg)
        self.get_logger().info(f'Published AI command: {ai_cmd_msg.command}')

    def sentiment_to_robot_action(self, result):
        """
        Convert sentiment analysis result to robot action.
        """
        label = result['label']
        score = result['score']
        
        # Map sentiment to robot actions
        if 'POS' in label:
            return {'command': 'move_forward', 'confidence': score}
        elif 'NEG' in label:
            return {'command': 'move_backward', 'confidence': score}
        else:  # NEUTRAL
            return {'command': 'idle', 'confidence': score}

def main(args=None):
    rclpy.init(args=args)
    node = HFTransformerNode()
    
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

### 3.5.2 Wrapping OpenAI API Calls

Here's an example of how to wrap OpenAI API calls in a ROS 2 node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from athena_interfaces.msg import AICommand
import openai  # You'll need to install openai
import os

class OpenAINode(Node):
    def __init__(self):
        super().__init__('openai_node')
        
        # Initialize OpenAI
        openai.api_key = os.getenv('OPENAI_API_KEY')
        if not openai.api_key:
            self.get_logger().error('OPENAI_API_KEY environment variable not set')
        
        # Subscribe to natural language commands
        self.command_subscription = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )
        
        # Publisher for AI-generated robot commands
        self.command_publisher = self.create_publisher(AICommand, 'ai_robot_command', 10)

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        
        # Create a prompt for the AI
        prompt = f"Convert the following human instruction into a robot command: '{msg.data}'. Respond with only one of these commands: move_forward, move_backward, turn_left, turn_right, wave_hand, raise_arms, lower_arms, nod_head, shake_head, idle, sit, stand."
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": "You are a robot command interpreter. Respond with only a simple command from the list."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=10,
                temperature=0.1  # Lower temperature for more consistent outputs
            )
            
            # Extract the robot command from the response
            robot_cmd = response.choices[0].message.content.strip().lower()
            
            # Create and publish the AI command
            ai_cmd_msg = AICommand()
            ai_cmd_msg.command = robot_cmd
            ai_cmd_msg.confidence = 0.9  # Assuming high confidence for GPT
            ai_cmd_msg.description = f"Generated from: {msg.data}"
            
            self.command_publisher.publish(ai_cmd_msg)
            self.get_logger().info(f'Published AI command: {ai_cmd_msg.command}')
        
        except Exception as e:
            self.get_logger().error(f'Error calling OpenAI API: {str(e)}')
            # Respond with idle command on error
            error_cmd = AICommand()
            error_cmd.command = 'idle'
            error_cmd.confidence = 0.1
            error_cmd.description = f"Error occurred: {str(e)}"
            self.command_publisher.publish(error_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = OpenAINode()
    
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

## 3.6 Latency Considerations and Performance Best Practices

When running AI models alongside real-time robot control, special considerations are needed to ensure the system performs well.

### 3.6.1 Latency Measurement Tools

Here's an example of how to measure latency in your AI-robot communication system:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
import time

class LatencyMeasurementNode(Node):
    def __init__(self):
        super().__init__('latency_measurement_node')
        
        # Publisher to send timestamped messages
        self.publisher = self.create_publisher(Header, 'latency_test', 10)
        
        # Subscription to receive timestamped echoes
        self.subscription = self.create_subscription(
            Header,
            'latency_test_echo',
            self.latency_response_callback,
            10
        )
        
        # Timer to periodically send test messages
        self.timer = self.create_timer(1.0, self.send_test_message)
        
        # Storage for tracking message timestamps
        self.sent_times = {}
        self.message_counter = 0
        
        self.get_logger().info('Latency measurement initialized')

    def send_test_message(self):
        """
        Send a test message with a timestamp to measure round-trip time.
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = f"latency_test_{self.message_counter}"
        
        # Record the time we sent the message
        self.sent_times[header.frame_id] = time.time()
        self.message_counter += 1
        
        self.publisher.publish(header)
        self.get_logger().info(f'Sent test message: {header.frame_id}')

    def latency_response_callback(self, msg):
        """
        Callback for receiving echoed messages and calculating latency.
        """
        # Get the time we received the message
        receive_time = time.time()
        
        # Look up the time we sent the message
        sent_time = self.sent_times.pop(msg.frame_id, None)
        
        if sent_time:
            latency = receive_time - sent_time
            self.get_logger().info(f'Round-trip latency for {msg.frame_id}: {latency:.4f} seconds')
            
            # Check if latency meets requirements (1. 0.1:  # 100ms
                self.get_logger().warn(f'Latency exceeded 100ms threshold: {latency:.4f}s')
        else:
            self.get_logger().warn(f'Received echo for unknown message: {msg.frame_id}')

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

### 3.6.2 Best Practices for Running LLMs with Real-time Control

When running large language models alongside real-time control systems, consider these best practices:

1. **Asynchronous Processing**: Use separate threads for AI computation to avoid blocking real-time control loops
2. **Resource Management**: Limit the computational resources allocated to AI models
3. **Caching**: Cache responses for frequently used commands
4. **Fallback Mechanisms**: Implement fallback behaviors when AI services are unavailable
5. **Throttling**: Throttle AI requests when the system is under heavy load

## 3.7 Pro Tips: Security and Best Practices

- **Secure API Keys**: Never hardcode API keys; use environment variables or secure credential management
- **Input Validation**: Validate and sanitize all inputs from AI models before sending to robot
- **Rate Limiting**: Implement rate limiting to prevent AI models from overwhelming the robot
- **Sandboxing**: Run AI models in a sandboxed environment to limit potential impacts
- **Error Handling**: Always implement proper error handling for API failures or network timeouts
- **Logging**: Monitor AI-robot interactions for debugging and security analysis

## 3.8 Summary

This chapter has covered how to bridge AI agents with robots using rclpy. We've seen how to create publishers for robot control, subscribers for sensor processing, and how to wrap powerful AI models like Hugging Face transformers and OpenAI API calls inside ROS 2 nodes. We've also discussed critical issues like latency considerations and performance best practices for running LLMs alongside real-time control systems.

These techniques form the foundation for creating intelligent robotic systems that can leverage advanced AI models while maintaining safety and reliability. In the following chapters, we'll look at how to model robots using URDF and Xacro, and then tie everything together into a complete ROS 2 package.

## Exercises

1. Create an AI node that processes camera images using a vision model and makes navigation decisions.
2. Implement a safety layer that validates AI-generated commands before execution.
3. Design a caching system for AI responses to improve system responsiveness.
4. Create a latency monitoring system for AI-robot communication.
5. Implement error handling for when the AI service becomes unavailable.

### Solutions to Exercises

[Detailed solutions would be provided in the exercises appendix]


