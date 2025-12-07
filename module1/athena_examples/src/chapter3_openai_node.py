#!/usr/bin/env python3

"""
Chapter 3: OpenAI API in ROS 2 Node Example
This example demonstrates how to wrap an OpenAI API call in a ROS 2 node.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from athena_interfaces.msg import AICommand  # Custom message type for AI commands
import threading
import time


class OpenAINode(Node):
    """
    A node that wraps an OpenAI API call in a ROS 2 node.
    NOTE: This is a template that would require the openai library and API key to function fully.
    """

    def __init__(self):
        super().__init__('openai_node')
        
        # For this template, we won't initialize the actual OpenAI API since we don't have credentials
        self.get_logger().info('OpenAI Node initialized (using mock responses for template)')
        
        # Subscription to natural language commands
        self.nlp_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.language_callback,
            10
        )
        self.nlp_sub  # Prevent unused variable warning
        
        # Publisher for AI commands to robot
        self.ai_command_publisher = self.create_publisher(AICommand, 'ai_robot_command', 10)
        
        self.get_logger().info('OpenAI Node initialized')

    def language_callback(self, msg):
        """
        Callback function that processes natural language with OpenAI API.
        """
        self.get_logger().info(f'Received natural language command: {msg.data}')
        
        # In a real implementation, we would call the OpenAI API:
        # response = openai.ChatCompletion.create(
        #     model="gpt-3.5-turbo",
        #     messages=[{"role": "user", "content": msg.data}],
        #     max_tokens=50
        # )
        # 
        # For this template, we'll use a mock response
        robot_cmd = self.mock_openai_api_call(msg.data)
        
        # Create and publish AI command
        ai_cmd = AICommand()
        ai_cmd.command = robot_cmd
        ai_cmd.confidence = 0.7  # For template purposes
        ai_cmd.description = f"Converted '{msg.data}' to '{robot_cmd}' via OpenAI API"
        
        self.ai_command_publisher.publish(ai_cmd)
        self.get_logger().info(f'Published AI command: {ai_cmd.command}')

    def mock_openai_api_call(self, command):
        """
        Mock implementation of OpenAI API call for demonstration.
        """
        # Simulate API call delay
        time.sleep(0.1)
        
        # Simple mapping for demonstration purposes (in reality, this would be the AI's interpretation)
        cmd_lower = command.lower()
        
        if 'forward' in cmd_lower or 'move ahead' in cmd_lower or 'go straight' in cmd_lower:
            return 'move_forward'
        elif 'backward' in cmd_lower or 'reverse' in cmd_lower or 'go back' in cmd_lower:
            return 'move_backward'
        elif 'turn left' in cmd_lower or 'rotate left' in cmd_lower:
            return 'turn_left'
        elif 'turn right' in cmd_lower or 'rotate right' in cmd_lower:
            return 'turn_right'
        elif 'wave' in cmd_lower or 'hello' in cmd_lower or 'greet' in cmd_lower:
            return 'wave_hand'
        elif 'pick up' in cmd_lower or 'grasp' in cmd_lower or 'take' in cmd_lower:
            return 'grasp_object'
        elif 'put down' in cmd_lower or 'place' in cmd_lower or 'release' in cmd_lower:
            return 'release_object'
        elif 'stop' in cmd_lower or 'halt' in cmd_lower or 'stand still' in cmd_lower:
            return 'emergency_stop'
        elif 'dance' in cmd_lower or 'party' in cmd_lower:
            return 'perform_dance'
        elif 'sit' in cmd_lower or 'sit down' in cmd_lower:
            return 'sit_down'
        elif 'stand' in cmd_lower or 'stand up' in cmd_lower:
            return 'stand_up'
        else:
            # Default to idle if command is unclear
            return 'idle'


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    rclpy.init(args=args)

    openai_node = OpenAINode()

    try:
        rclpy.spin(openai_node)
    except KeyboardInterrupt:
        pass
    finally:
        openai_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()