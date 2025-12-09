# Chapter 19: Real-World Deployment – Perception, Execution, and Safety

## Learning Objectives

After completing this chapter, you will be able to:
- Deploy VLA systems safely in unstructured real-world environments
- Implement comprehensive safety systems with emergency protocols
- Calibrate action spaces between simulation and real hardware
- Handle perception challenges in real-world settings
- Implement error recovery mechanisms for robust operation

## 19.1 Real-World Perception Challenges

Real-world environments present numerous challenges for perception systems, including variable lighting, cluttered scenes, and dynamic obstacles that are not present in simulated environments.

## 19.2 Action Space Calibration and Mapping

Transferring policies from simulation to real hardware requires precise calibration of action spaces to account for differences in kinematics, dynamics, and sensor configurations.

Let's implement the hardware abstraction layer:

```python
# module4/chapter19/code/hardware_abstraction.py
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np

class HardwareAbstractionLayer:
    def __init__(self, robot_name="athena", use_real_hardware=True):
        self.robot_name = robot_name
        self.use_real_hardware = use_real_hardware
        
        if use_real_hardware:
            # Initialize ROS node for real hardware
            rospy.init_node('vla_hardware_interface', anonymous=True)
            
            # Create publishers for various robot components
            self.joint_pub = rospy.Publisher(
                f'/{robot_name}/joint_commands', 
                JointState, 
                queue_size=10
            )
            self.cmd_vel_pub = rospy.Publisher(
                f'/{robot_name}/cmd_vel', 
                Twist, 
                queue_size=10
            )
            
            # Create subscribers for sensor feedback
            self.joint_sub = rospy.Subscriber(
                f'/{robot_name}/joint_states', 
                JointState, 
                self.joint_state_callback
            )
            
            self.current_joint_states = None
    
    def joint_state_callback(self, data):
        """
        Callback for joint state updates
        """
        self.current_joint_states = data
    
    def execute_action(self, action_vector, action_space_type="joint_positions"):
        """
        Execute action on real hardware with safety checks
        """
        if not self.verify_action_safety(action_vector, action_space_type):
            raise ValueError("Action failed safety verification")
        
        if action_space_type == "joint_positions":
            return self.execute_joint_position_command(action_vector)
        elif action_space_type == "joint_velocities":
            return self.execute_joint_velocity_command(action_vector)
        elif action_space_type == "cartesian":
            return self.execute_cartesian_command(action_vector)
        else:
            raise ValueError(f"Unknown action space type: {action_space_type}")
    
    def verify_action_safety(self, action_vector, action_space_type):
        """
        Verify that the action is safe to execute
        """
        # Check joint limits
        if action_space_type == "joint_positions":
            # Implement joint limit checking
            return self.check_joint_limits(action_vector)
        
        return True  # Simplified safety check
    
    def check_joint_limits(self, joint_positions):
        """
        Check if joint positions are within safe limits
        """
        # Define joint limits (simplified approach)
        min_limits = [-3.14] * len(joint_positions)  # Example limits
        max_limits = [3.14] * len(joint_positions)   # Example limits
        
        for i, pos in enumerate(joint_positions):
            if pos < min_limits[i] or pos > max_limits[i]:
                return False
        
        return True
    
    def execute_joint_position_command(self, joint_positions):
        """
        Execute joint position command
        """
        if self.use_real_hardware:
            joint_msg = JointState()
            joint_msg.position = joint_positions
            joint_msg.header.stamp = rospy.Time.now()
            self.joint_pub.publish(joint_msg)
        else:
            # Simulation implementation would go here
            print(f"Simulated execution of joint positions: {joint_positions}")
        
        return True