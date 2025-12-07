#!/usr/bin/env python3

"""
Chapter 3: Joint Trajectory Publisher
This example demonstrates using rclpy to publish joint trajectories to control a robot.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class JointTrajectoryPublisher(Node):
    """
    A node that publishes joint trajectories to control robot joints.
    """

    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        
        # Create publisher for joint trajectories
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Timer to periodically send trajectories
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Define joint names for the "athena" humanoid
        self.joint_names = [
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]
        
        # Counter for trajectory generation
        self.trajectory_counter = 0
        
        self.get_logger().info('Joint Trajectory Publisher initialized')

    def timer_callback(self):
        """
        Callback function that generates and publishes joint trajectories.
        """
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Create trajectory points
        points = []
        
        # For demonstration, we'll create a simple oscillating trajectory
        for i in range(3):  # Creating 3 points for smooth motion
            point = JointTrajectoryPoint()
            
            # Set positions based on a sine wave pattern
            time_offset = self.trajectory_counter + i
            positions = []
            for idx, _ in enumerate(self.joint_names):
                # Different joints have different oscillation patterns
                pos = math.sin(time_offset * 0.5 + idx) * 0.5
                positions.append(pos)
            
            point.positions = positions
            point.velocities = [0.0] * len(self.joint_names)
            point.accelerations = [0.0] * len(self.joint_names)
            point.effort = [0.0] * len(self.joint_names)
            
            # Set timing - each point spaced 0.5 seconds apart
            point.time_from_start = Duration(sec=i, nanosec=int(500000000 * (i / 3)))
            
            points.append(point)
        
        msg.points = points
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published trajectory with {len(points)} points for {len(self.joint_names)} joints')
        
        self.trajectory_counter += 1


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    rclpy.init(args=args)

    trajectory_publisher = JointTrajectoryPublisher()

    try:
        rclpy.spin(trajectory_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        trajectory_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()