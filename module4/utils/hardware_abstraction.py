"""
Hardware abstraction layer for Module 4
Provides a unified interface for controlling real and simulated robots
"""
import rospy
import numpy as np
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Twist, Pose
from typing import Dict, List, Tuple, Optional, Any
from abc import ABC, abstractmethod
import logging
import time

logger = logging.getLogger(__name__)

class RobotInterface(ABC):
    """
    Abstract base class for robot interfaces
    Provides a common interface for both simulated and real robots
    """
    
    @abstractmethod
    def connect(self) -> bool:
        """Connect to the robot"""
        pass
    
    @abstractmethod
    def disconnect(self) -> bool:
        """Disconnect from the robot"""
        pass
    
    @abstractmethod
    def execute_action(self, action: List[float], action_type: str = "joint_positions") -> bool:
        """Execute an action on the robot"""
        pass
    
    @abstractmethod
    def get_joint_states(self) -> List[float]:
        """Get current joint states"""
        pass
    
    @abstractmethod
    def get_robot_pose(self) -> Pose:
        """Get current robot pose"""
        pass
    
    @abstractmethod
    def is_connected(self) -> bool:
        """Check if robot is connected"""
        pass


class SimulatedRobotInterface(RobotInterface):
    """
    Simulated robot interface for testing and development
    """
    
    def __init__(self, robot_name: str = "athena_sim"):
        self.robot_name = robot_name
        self._is_connected = False
        self._joint_states = [0.0] * 7  # Default joint positions
        self._robot_pose = Pose()  # Default pose
        self._action_history = []
    
    def connect(self) -> bool:
        """Connect to the simulated robot"""
        logger.info(f"Connecting to simulated robot: {self.robot_name}")
        self._is_connected = True
        logger.info("Connected to simulated robot successfully")
        return True
    
    def disconnect(self) -> bool:
        """Disconnect from the simulated robot"""
        logger.info(f"Disconnecting from simulated robot: {self.robot_name}")
        self._is_connected = False
        logger.info("Disconnected from simulated robot")
        return True
    
    def execute_action(self, action: List[float], action_type: str = "joint_positions") -> bool:
        """Execute an action on the simulated robot"""
        if not self._is_connected:
            logger.error("Cannot execute action: robot not connected")
            return False
        
        try:
            logger.info(f"Executing {action_type} action on simulated robot: {action}")
            
            if action_type == "joint_positions":
                # Validate joint positions
                if len(action) != len(self._joint_states):
                    logger.warning(f"Action length ({len(action)}) does not match joint states length ({len(self._joint_states)}), padding or truncating")
                    action = action[:len(self._joint_states)] + [0.0] * max(0, len(self._joint_states) - len(action))
                
                # Update joint states
                self._joint_states = action.copy()
            elif action_type == "joint_velocities":
                # Update joint states based on velocity
                dt = 0.1  # Time step for simulation
                for i in range(len(self._joint_states)):
                    self._joint_states[i] += action[i] * dt
            elif action_type == "cartesian":
                # For simulation, treat cartesian as direct pose update
                logger.info("Simulated cartesian action execution")
            else:
                logger.error(f"Unknown action type: {action_type}")
                return False
            
            # Log the action
            self._action_history.append({
                'action': action.copy(),
                'action_type': action_type,
                'timestamp': time.time()
            })
            
            logger.info(f"Action executed successfully on simulated robot")
            return True
            
        except Exception as e:
            logger.error(f"Error executing action on simulated robot: {e}")
            return False
    
    def get_joint_states(self) -> List[float]:
        """Get current joint states from simulated robot"""
        if not self._is_connected:
            logger.warning("Robot not connected, returning default joint states")
            return [0.0] * 7
        
        return self._joint_states.copy()
    
    def get_robot_pose(self) -> Pose:
        """Get current robot pose from simulated robot"""
        if not self._is_connected:
            logger.warning("Robot not connected, returning default pose")
            return Pose()
        
        return self._robot_pose
    
    def is_connected(self) -> bool:
        """Check if simulated robot is connected"""
        return self._is_connected


class RealRobotInterface(RobotInterface):
    """
    Real robot interface for controlling actual hardware
    """
    
    def __init__(self, robot_name: str = "athena", namespace: str = ""):
        self.robot_name = robot_name
        self.namespace = namespace
        self._is_connected = False
        
        # ROS publishers and subscribers
        self.joint_pub = None
        self.cmd_vel_pub = None
        self.joint_sub = None
        self.pose_sub = None
        
        # Current robot state
        self._current_joint_states = None
        self._current_pose = None
        
        # Safety parameters
        self._max_joint_velocity = 1.0  # rad/s
        self._max_cartesian_velocity = 0.5  # m/s
        self._joint_limits = {
            'min': [-3.14] * 7,  # Example joint limits
            'max': [3.14] * 7
        }
    
    def connect(self) -> bool:
        """Connect to the real robot via ROS"""
        try:
            logger.info(f"Connecting to real robot: {self.robot_name}")
            
            # Initialize ROS node if not already initialized
            if not rospy.get_node_uri():
                rospy.init_node(f'{self.robot_name}_interface', anonymous=True)
            
            # Create publishers for robot control
            joint_topic = f'/{self.robot_name}/joint_commands' if self.namespace else f'/joint_commands'
            self.joint_pub = rospy.Publisher(joint_topic, Float64MultiArray, queue_size=10)
            
            cmd_vel_topic = f'/{self.robot_name}/cmd_vel' if self.namespace else f'/cmd_vel'
            self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
            
            # Create subscribers for robot feedback
            joint_state_topic = f'/{self.robot_name}/joint_states' if self.namespace else f'/joint_states'
            self.joint_sub = rospy.Subscriber(joint_state_topic, JointState, self._joint_state_callback)
            
            pose_topic = f'/{self.robot_name}/pose' if self.namespace else f'/pose'
            # self.pose_sub = rospy.Subscriber(pose_topic, Pose, self._pose_callback)  # Uncomment if pose topic exists
            
            # Wait a moment for connections to establish
            rospy.sleep(1.0)
            
            self._is_connected = True
            logger.info(f"Connected to real robot: {self.robot_name}")
            return True
            
        except Exception as e:
            logger.error(f"Error connecting to real robot: {e}")
            return False
    
    def disconnect(self) -> bool:
        """Disconnect from the real robot"""
        logger.info(f"Disconnecting from real robot: {self.robot_name}")
        
        # In ROS, we typically don't explicitly disconnect publishers/subscribers
        # The node shutdown is handled by ROS when the process ends
        self._is_connected = False
        logger.info("Disconnected from real robot")
        return True
    
    def execute_action(self, action: List[float], action_type: str = "joint_positions") -> bool:
        """Execute an action on the real robot with safety checks"""
        if not self._is_connected:
            logger.error("Cannot execute action: robot not connected")
            return False
        
        try:
            logger.info(f"Executing {action_type} action on real robot: {action}")
            
            # Validate action based on type
            if not self._validate_action(action, action_type):
                logger.error(f"Action validation failed for {action_type}: {action}")
                return False
            
            # Safety check for joint limits
            if action_type == "joint_positions" and not self._check_joint_limits(action):
                logger.error(f"Action violates joint limits: {action}")
                return False
            
            # Execute based on action type
            if action_type == "joint_positions":
                return self._execute_joint_position_command(action)
            elif action_type == "joint_velocities":
                return self._execute_joint_velocity_command(action)
            elif action_type == "cartesian":
                return self._execute_cartesian_command(action)
            else:
                logger.error(f"Unknown action type: {action_type}")
                return False
                
        except Exception as e:
            logger.error(f"Error executing action on real robot: {e}")
            return False
    
    def get_joint_states(self) -> List[float]:
        """Get current joint states from real robot"""
        if not self._is_connected:
            logger.warning("Robot not connected, returning empty joint states")
            return []
        
        if self._current_joint_states is not None:
            return list(self._current_joint_states)
        else:
            logger.warning("No joint state data available yet")
            return []
    
    def get_robot_pose(self) -> Pose:
        """Get current robot pose from real robot"""
        if not self._is_connected:
            logger.warning("Robot not connected, returning default pose")
            return Pose()
        
        if self._current_pose is not None:
            return self._current_pose
        else:
            logger.warning("No pose data available yet")
            return Pose()
    
    def is_connected(self) -> bool:
        """Check if real robot is connected"""
        return self._is_connected
    
    def _validate_action(self, action: List[float], action_type: str) -> bool:
        """Validate action parameters"""
        if not action:
            logger.error("Action is empty")
            return False
        
        if action_type == "joint_positions" or action_type == "joint_velocities":
            # For now, assume 7-DOF robot, but this should be configurable
            if len(action) != 7:
                logger.warning(f"Expected 7 DOF for {action_type}, got {len(action)}. This may be OK depending on robot config.")
        
        return True
    
    def _check_joint_limits(self, joint_positions: List[float]) -> bool:
        """Check if joint positions are within safe limits"""
        for i, pos in enumerate(joint_positions):
            if pos < self._joint_limits['min'][i] or pos > self._joint_limits['max'][i]:
                logger.warning(f"Joint {i} position {pos} exceeds limits [{self._joint_limits['min'][i]}, {self._joint_limits['max'][i]}]")
                return False
        return True
    
    def _execute_joint_position_command(self, joint_positions: List[float]) -> bool:
        """Execute joint position command on real robot"""
        if self.joint_pub is None:
            logger.error("Joint publisher not initialized")
            return False
        
        # Create and publish joint command
        joint_cmd = Float64MultiArray()
        joint_cmd.data = joint_positions
        self.joint_pub.publish(joint_cmd)
        
        logger.info(f"Published joint positions: {joint_positions}")
        return True
    
    def _execute_joint_velocity_command(self, joint_velocities: List[float]) -> bool:
        """Execute joint velocity command on real robot"""
        if self.joint_pub is None:
            logger.error("Joint publisher not initialized")
            return False
        
        # Check velocity limits
        for vel in joint_velocities:
            if abs(vel) > self._max_joint_velocity:
                logger.warning(f"Joint velocity {vel} exceeds max {self._max_joint_velocity}, clipping")
        
        # Create and publish velocity command
        vel_cmd = Float64MultiArray()
        clipped_velocities = [max(-self._max_joint_velocity, min(self._max_joint_velocity, v)) for v in joint_velocities]
        vel_cmd.data = clipped_velocities
        self.joint_pub.publish(vel_cmd)
        
        logger.info(f"Published joint velocities: {clipped_velocities}")
        return True
    
    def _execute_cartesian_command(self, cartesian_action: List[float]) -> bool:
        """Execute cartesian command on real robot"""
        if self.cmd_vel_pub is None:
            logger.error("Command velocity publisher not initialized")
            return False
        
        # Convert cartesian action to Twist message
        # Assuming cartesian_action = [vx, vy, vz, rx, ry, rz] for linear and angular velocities
        twist_cmd = Twist()
        twist_cmd.linear.x = cartesian_action[0] if len(cartesian_action) > 0 else 0.0
        twist_cmd.linear.y = cartesian_action[1] if len(cartesian_action) > 1 else 0.0
        twist_cmd.linear.z = cartesian_action[2] if len(cartesian_action) > 2 else 0.0
        twist_cmd.angular.x = cartesian_action[3] if len(cartesian_action) > 3 else 0.0
        twist_cmd.angular.y = cartesian_action[4] if len(cartesian_action) > 4 else 0.0
        twist_cmd.angular.z = cartesian_action[5] if len(cartesian_action) > 5 else 0.0
        
        # Apply velocity limits
        linear_mag = (twist_cmd.linear.x**2 + twist_cmd.linear.y**2 + twist_cmd.linear.z**2)**0.5
        if linear_mag > self._max_cartesian_velocity:
            scale = self._max_cartesian_velocity / linear_mag
            twist_cmd.linear.x *= scale
            twist_cmd.linear.y *= scale
            twist_cmd.linear.z *= scale
        
        self.cmd_vel_pub.publish(twist_cmd)
        
        logger.info(f"Published cartesian command: linear=({twist_cmd.linear.x}, {twist_cmd.linear.y}, {twist_cmd.linear.z}), angular=({twist_cmd.angular.x}, {twist_cmd.angular.y}, {twist_cmd.angular.z})")
        return True
    
    def _joint_state_callback(self, msg: JointState):
        """Callback for joint state updates"""
        self._current_joint_states = list(msg.position)
    
    def _pose_callback(self, msg: Pose):
        """Callback for pose updates"""
        self._current_pose = msg


class HardwareAbstractionLayer:
    """
    Main hardware abstraction layer that selects between simulated and real hardware
    """
    
    def __init__(self, 
                 robot_name: str = "athena", 
                 use_real_hardware: bool = False, 
                 namespace: str = "",
                 safety_enabled: bool = True):
        """
        Initialize hardware abstraction layer
        
        Args:
            robot_name: Name of the robot
            use_real_hardware: Whether to connect to real hardware or use simulation
            namespace: ROS namespace for the robot
            safety_enabled: Whether to enforce safety checks
        """
        self.robot_name = robot_name
        self.use_real_hardware = use_real_hardware
        self.namespace = namespace
        self.safety_enabled = safety_enabled
        
        # Initialize the appropriate interface
        if use_real_hardware:
            self.robot_interface = RealRobotInterface(robot_name, namespace)
        else:
            self.robot_interface = SimulatedRobotInterface(robot_name)
        
        logger.info(f"Hardware abstraction layer initialized for robot: {robot_name}, real hardware: {use_real_hardware}")
    
    def connect(self) -> bool:
        """Connect to the robot"""
        return self.robot_interface.connect()
    
    def disconnect(self) -> bool:
        """Disconnect from the robot"""
        return self.robot_interface.disconnect()
    
    def execute_action(self, action: List[float], action_type: str = "joint_positions") -> bool:
        """Execute an action on the robot"""
        # In a real implementation, additional safety checks could be added here
        if self.safety_enabled:
            # Additional safety validation could go here
            pass
        
        return self.robot_interface.execute_action(action, action_type)
    
    def get_joint_states(self) -> List[float]:
        """Get current joint states"""
        return self.robot_interface.get_joint_states()
    
    def get_robot_pose(self) -> Pose:
        """Get current robot pose"""
        return self.robot_interface.get_robot_pose()
    
    def is_connected(self) -> bool:
        """Check if robot is connected"""
        return self.robot_interface.is_connected()
    
    def emergency_stop(self) -> bool:
        """Execute emergency stop"""
        logger.warning("Emergency stop activated!")
        
        # Send zero commands to stop all robot motion
        zero_action = [0.0] * 7  # Assuming 7-DOF robot
        return self.robot_interface.execute_action(zero_action, "joint_velocities")
    
    def get_robot_info(self) -> Dict[str, Any]:
        """Get information about the robot"""
        info = {
            'robot_name': self.robot_name,
            'use_real_hardware': self.use_real_hardware,
            'namespace': self.namespace,
            'is_connected': self.robot_interface.is_connected(),
            'safety_enabled': self.safety_enabled
        }
        
        if self.robot_interface.is_connected():
            info['joint_states'] = self.robot_interface.get_joint_states()
            info['robot_pose'] = self.robot_interface.get_robot_pose()
        
        return info


# Standalone function for easy usage
def initialize_hardware_abstraction(robot_name: str = "athena", use_real_hardware: bool = False) -> HardwareAbstractionLayer:
    """
    Initialize and return hardware abstraction layer
    
    Args:
        robot_name: Name of the robot to control
        use_real_hardware: Whether to connect to real hardware or use simulation
        
    Returns:
        Initialized HardwareAbstractionLayer instance
    """
    return HardwareAbstractionLayer(robot_name, use_real_hardware)


if __name__ == "__main__":
    # Example usage
    print("Testing Hardware Abstraction Layer...")
    
    # Initialize hardware abstraction layer with simulation
    hal = initialize_hardware_abstraction("athena", use_real_hardware=False)
    print("Hardware Abstraction Layer initialized successfully!")
    
    # Connect to robot
    connected = hal.connect()
    print(f"Connected to robot: {connected}")
    
    # Test action execution
    if connected:
        test_action = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        action_success = hal.execute_action(test_action, "joint_positions")
        print(f"Action execution success: {action_success}")
        
        # Get current state
        joint_states = hal.get_joint_states()
        print(f"Current joint states: {joint_states}")
    
    # Disconnect
    disconnected = hal.disconnect()
    print(f"Disconnected from robot: {disconnected}")
    
    print("Hardware Abstraction Layer test completed.")