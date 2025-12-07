#!/usr/bin/env python3

"""
Chapter 3: Basic rclpy Node Example
This example demonstrates the fundamentals of creating an rclpy node for AI-robot interaction.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class BasicAINode(Node):
    """
    A basic AI node that demonstrates fundamental rclpy concepts.
    """

    def __init__(self):
        super().__init__('basic_ai_node')
        
        # Create a publisher
        self.publisher_ = self.create_publisher(String, 'ai_commands', 10)
        
        # Timer for periodic publishing
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Counter for message numbering
        self.i = 0
        
        self.get_logger().info('Basic AI Node initialized')

    def timer_callback(self):
        """
        Callback function that is called periodically to publish messages.
        """
        msg = String()
        msg.data = f'AI Command: {self.i}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        
        self.i += 1


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    rclpy.init(args=args)

    basic_ai_node = BasicAINode()

    try:
        rclpy.spin(basic_ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        basic_ai_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()