#!/usr/bin/env python3

"""
Chapter 5: Waving Motion Demonstration
This example demonstrates how to publish a JointTrajectory command to make the "athena" humanoid robot wave.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class WavingDemoNode(Node):
    """
    A node that demonstrates publishing JointTrajectory messages to make the robot wave.
    """

    def __init__(self):
        super().__init__('waving_demo_node')
        
        # Publisher for joint trajectories
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Timer to send waving motion periodically
        self.timer = self.create_timer(5.0, self.send_waving_motion)
        
        # Define joint names for the "athena" humanoid (right arm for waving)
        self.joint_names = [
            'right_shoulder_yaw', 
            'right_elbow_pitch'
        ]
        
        self.get_logger().info('Waving Demo Node initialized - will make robot wave every 5 seconds')

    def send_waving_motion(self):
        """
        Send a JointTrajectory command that makes the robot wave.
        """
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Create trajectory points for waving motion
        points = []
        
        # Point 1: Starting position (arm at side)
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0]  # Shoulder and elbow at neutral
        point1.velocities = [0.0, 0.0]
        point1.accelerations = [0.0, 0.0]
        point1.time_from_start = Duration(sec=0, nanosec=0)
        points.append(point1)
        
        # Point 2: Raise arm to waving position
        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, 0.8]  # Lift shoulder, bend elbow
        point2.velocities = [0.0, 0.0]
        point2.accelerations = [0.0, 0.0]
        point2.time_from_start = Duration(sec=1, nanosec=0)
        points.append(point2)
        
        # Point 3: Wave up
        point3 = JointTrajectoryPoint()
        point3.positions = [0.3, 1.2]  # Adjust shoulder and elbow for wave up
        point3.velocities = [0.0, 0.0]
        point3.accelerations = [0.0, 0.0]
        point3.time_from_start = Duration(sec=1.5, nanosec=0)
        points.append(point3)
        
        # Point 4: Wave down (return to center)
        point4 = JointTrajectoryPoint()
        point4.positions = [0.5, 0.4]  # Back to center position
        point4.velocities = [0.0, 0.0]
        point4.accelerations = [0.0, 0.0]
        point4.time_from_start = Duration(sec=2.0, nanosec=0)
        points.append(point4)
        
        # Point 5: Wave up again (second wave)
        point5 = JointTrajectoryPoint()
        point5.positions = [0.3, 1.2]  # Up again
        point5.velocities = [0.0, 0.0]
        point5.accelerations = [0.0, 0.0]
        point5.time_from_start = Duration(sec=2.5, nanosec=0)
        points.append(point5)
        
        # Point 6: Return to center
        point6 = JointTrajectoryPoint()
        point6.positions = [0.5, 0.4]  # Back to center
        point6.velocities = [0.0, 0.0]
        point6.accelerations = [0.0, 0.0]
        point6.time_from_start = Duration(sec=3.0, nanosec=0)
        points.append(point6)
        
        # Point 7: Lower arm back to side
        point7 = JointTrajectoryPoint()
        point7.positions = [0.0, 0.0]  # Back to neutral
        point7.velocities = [0.0, 0.0]
        point7.accelerations = [0.0, 0.0]
        point7.time_from_start = Duration(sec=4.0, nanosec=0)
        points.append(point7)
        
        msg.points = points
        self.joint_trajectory_publisher.publish(msg)
        
        self.get_logger().info(f'Published waving trajectory with {len(points)} points')


def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    rclpy.init(args=args)

    waving_demo = WavingDemoNode()

    try:
        rclpy.spin(waving_demo)
    except KeyboardInterrupt:
        pass
    finally:
        waving_demo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()