#!/usr/bin/env python3

"""
Chapter 3: Sensor Data Subscriber Example
This example demonstrates how to use rclpy to subscribe to sensor data from a robot.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import numpy as np


class SensorDataSubscriber(Node):
    """
    A node that subscribes to sensor data and processes it.
    """

    def __init__(self):
        super().__init__('sensor_data_subscriber')
        
        # Create subscription to joint states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        
        # Publisher for processed sensor data (for demonstration)
        self.processed_publisher = self.create_publisher(String, 'processed_sensor_data', 10)
        
        # Initialize variables to store sensor data
        self.last_joint_states = None
        self.get_logger().info('Sensor Data Subscriber initialized')

    def joint_state_callback(self, msg):
        """
        Callback function that processes incoming sensor data.
        """
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')
        
        # Store the joint states
        self.last_joint_states = msg
        
        # Process the sensor data if there are positions
        if len(msg.position) > 0:
            # Calculate some statistics
            avg_position = np.mean(msg.position)
            pos_std = np.std(msg.position)
            
            # Simple AI decision based on sensor data
            ai_decision = self.make_decision(avg_position, pos_std)
            
            # Publish the processed data
            processed_msg = String()
            processed_msg.data = f"Decision: {ai_decision}, Avg Pos: {avg_position:.3f}, Std: {pos_std:.3f}"
            self.processed_publisher.publish(processed_msg)
            
            self.get_logger().info(f'Processed data: {processed_msg.data}')
        else:
            self.get_logger().warn('Received joint states with no position data')

    def make_decision(self, avg_position, std_deviation):
        """
        Simple AI decision making based on sensor data.
        """
        if abs(avg_position) > 1.0:
            return "Adjust joint positions"
        elif std_deviation > 0.5:
            return "Check for anomalies"
        else:
            return "Continue normal operation"


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    rclpy.init(args=args)

    sensor_subscriber = SensorDataSubscriber()

    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()