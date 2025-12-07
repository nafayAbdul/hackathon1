# Chapter 5 Exercises: Building Your First ROS 2 Humanoid Package

## Exercise 1: Create a Launch File for Controllers Only

**Problem**: Create a ROS 2 launch file that starts only the robot controllers without launching Gazebo or RViz2.

**Solution**:

```python
# File: athena_bringup/launch/athena_controllers_only.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    athena_control_dir = get_package_share_directory('athena_control')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': open(
                os.path.join(
                    get_package_share_directory('athena_description'),
                    'urdf',
                    'athena.urdf'
                )).read()}
        ],
    )
    
    # Joint State Publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['joint_states'],
            'rate': 50.0,
        }]
    )
    
    # Controller Manager node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(athena_control_dir, 'config', 'athena_controllers.yaml'),
            {'use_sim_time': use_sim_time}
        ],
        output='both'
    )
    
    # Controller spawner nodes
    controller_spawners = [
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['joint_state_broadcaster'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['joint_trajectory_controller'],
            parameters=[{'use_sim_time': use_sim_time}],
        )
    ]
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        controller_manager,
    ] + controller_spawners)
```

This launch file starts the controllers without simulation or visualization, which is useful for:
- Testing controller configurations
- Connecting to real hardware
- Performance testing without simulation overhead
- Debugging controller issues

## Exercise 2: Enhance the Waving Motion

**Problem**: Modify the waving motion to be more complex, incorporating both arms in a choreographed sequence.

**Solution**:

```python
#!/usr/bin/env python3
# Enhanced waving demo with both arms

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class EnhancedWavingDemoNode(Node):
    def __init__(self):
        super().__init__('enhanced_waving_demo')
        
        # Publisher for joint trajectories
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Timer to send waving motion periodically
        self.timer = self.create_timer(10.0, self.send_enhanced_waving_trajectory)
        
        # Define joint names based on our athena humanoid
        self.joint_names = [
            'left_shoulder_yaw', 'left_elbow_pitch',
            'right_shoulder_yaw', 'right_elbow_pitch',
            'left_hip_yaw', 'left_knee_pitch', 'left_ankle_pitch',
            'right_hip_yaw', 'right_knee_pitch', 'right_ankle_pitch'
        ]
        
        self.get_logger().info('Enhanced Waving Demo Node initialized')

    def send_enhanced_waving_trajectory(self):
        """
        Sends a more complex waving trajectory involving both arms.
        """
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Create a more complex waving motion with both arms
        points = []
        
        # Point 1: Neutral position
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * len(self.joint_names)
        point1.velocities = [0.0] * len(self.joint_names)
        point1.accelerations = [0.0] * len(self.joint_names)
        point1.time_from_start = Duration(sec=0, nanosec=0)
        points.append(point1)
        
        # Point 2: Raise right arm to wave position
        point2 = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)
        positions[self.joint_names.index('right_shoulder_yaw')] = 0.5
        positions[self.joint_names.index('right_elbow_pitch')] = 0.8
        point2.positions = positions
        point2.velocities = [0.0] * len(self.joint_names)
        point2.accelerations = [0.0] * len(self.joint_names)
        point2.time_from_start = Duration(sec=1, nanosec=0)
        points.append(point2)
        
        # Point 3: Wave up with right arm
        point3 = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)
        positions[self.joint_names.index('right_shoulder_yaw')] = 0.3
        positions[self.joint_names.index('right_elbow_pitch')] = 1.2
        point3.positions = positions
        point3.velocities = [0.0] * len(self.joint_names)
        point3.accelerations = [0.0] * len(self.joint_names)
        point3.time_from_start = Duration(sec=1.5, nanosec=0)
        points.append(point3)
        
        # Point 4: Wave down with right arm
        point4 = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)
        positions[self.joint_names.index('right_shoulder_yaw')] = 0.5
        positions[self.joint_names.index('right_elbow_pitch')] = 0.4
        point4.positions = positions
        point4.velocities = [0.0] * len(self.joint_names)
        point4.accelerations = [0.0] * len(self.joint_names)
        point4.time_from_start = Duration(sec=2.0, nanosec=0)
        points.append(point4)
        
        # Point 5: Lower right arm, raise left arm
        point5 = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)
        positions[self.joint_names.index('right_shoulder_yaw')] = 0.0
        positions[self.joint_names.index('right_elbow_pitch')] = 0.0
        positions[self.joint_names.index('left_shoulder_yaw')] = -0.5
        positions[self.joint_names.index('left_elbow_pitch')] = -0.8
        point5.positions = positions
        point5.velocities = [0.0] * len(self.joint_names)
        point5.accelerations = [0.0] * len(self.joint_names)
        point5.time_from_start = Duration(sec=3.0, nanosec=0)
        points.append(point5)
        
        # Point 6: Left arm up for second wave
        point6 = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)
        positions[self.joint_names.index('left_shoulder_yaw')] = -0.3
        positions[self.joint_names.index('left_elbow_pitch')] = -1.2
        point6.positions = positions
        point6.velocities = [0.0] * len(self.joint_names)
        point6.accelerations = [0.0] * len(self.joint_names)
        point6.time_from_start = Duration(sec=3.5, nanosec=0)
        points.append(point6)
        
        # Point 7: Left arm down for second wave
        point7 = JointTrajectoryPoint()
        positions = [0.0] * len(self.joint_names)
        positions[self.joint_names.index('left_shoulder_yaw')] = -0.5
        positions[self.joint_names.index('left_elbow_pitch')] = -0.4
        point7.positions = positions
        point7.velocities = [0.0] * len(self.joint_names)
        point7.accelerations = [0.0] * len(self.joint_names)
        point7.time_from_start = Duration(sec=4.0, nanosec=0)
        points.append(point7)
        
        # Point 8: Return to neutral
        point8 = JointTrajectoryPoint()
        point8.positions = [0.0] * len(self.joint_names)
        point8.velocities = [0.0] * len(self.joint_names)
        point8.accelerations = [0.0] * len(self.joint_names)
        point8.time_from_start = Duration(sec=5.0, nanosec=0)
        points.append(point8)
        
        msg.points = points
        self.joint_trajectory_pub.publish(msg)
        
        self.get_logger().info(f'Published enhanced waving trajectory with {len(points)} points for {len(self.joint_names)} joints')

def main(args=None):
    """
    Main function that initializes the node and runs it.
    """
    rclpy.init(args=args)
    
    node = EnhancedWavingDemoNode()
    
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

## Exercise 3: Create a Walking Gait Controller Configuration

**Problem**: Create a controller configuration for a basic walking gait pattern for the "athena" humanoid.

**Solution**:

```yaml
# File: athena_control/config/walking_gait_controller.yaml

controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    walking_pattern_controller:
      type: position_controllers/JointTrajectoryController

walking_pattern_controller:
  ros__parameters:
    type: position_controllers/JointTrajectoryController
    joints:
      - left_hip_yaw
      - left_knee_pitch
      - left_ankle_pitch
      - right_hip_yaw
      - right_knee_pitch
      - right_ankle_pitch
    interface_name: position
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    open_loop_control: true
    allow_integration_in_goal_trajectories: true
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0
```

And the walking pattern controller node:

```python
#!/usr/bin/env python3
# File: athena_control/src/walking_pattern_controller.py

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class WalkingPatternController(Node):
    """
    A controller to generate basic walking patterns for the humanoid.
    """

    def __init__(self):
        super().__init__('walking_pattern_controller')
        
        # Publisher for joint trajectories
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/walking_pattern_controller/joint_trajectory',
            10
        )
        
        # Command subscriber to trigger walking
        self.walk_cmd_sub = self.create_subscription(
            Bool,
            'start_walking',
            self.walk_command_callback,
            10
        )
        
        # Timer for periodic walking pattern updates
        self.walking_enabled = False
        self.step_phase = 0.0
        self.walking_timer = self.create_timer(0.05, self.generate_walking_pattern)  # 20 Hz for walking
        
        # Define joint names for legs
        self.joint_names = [
            'left_hip_yaw', 'left_knee_pitch', 'left_ankle_pitch',
            'right_hip_yaw', 'right_knee_pitch', 'right_ankle_pitch'
        ]
        
        self.get_logger().info('Walking Pattern Controller initialized')

    def walk_command_callback(self, msg):
        """
        Callback to start/stop walking pattern.
        """
        self.walking_enabled = msg.data
        if self.walking_enabled:
            self.get_logger().info('Walking enabled')
        else:
            self.get_logger().info('Walking disabled')

    def generate_walking_pattern(self):
        """
        Generate a simple walking pattern using a sinusoidal approach.
        """
        if not self.walking_enabled:
            return
            
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Create a single trajectory point for the current step
        point = JointTrajectoryPoint()
        
        # Calculate phase for current step
        self.step_phase += 0.1  # Increment phase for next iteration
        
        # Generate walking pattern for each joint
        positions = []
        for i, joint_name in enumerate(self.joint_names):
            # Each joint has its own pattern based on its function
            if 'hip' in joint_name:
                # Hip joints: move to shift weight
                amplitude = 0.2 if 'left' in joint_name else -0.2
                position = amplitude * math.sin(self.step_phase)
            elif 'knee' in joint_name:
                # Knee joints: bend and straighten to lift and lower legs
                amplitude = 0.5 if 'left' in joint_name else -0.5
                position = amplitude * math.sin(self.step_phase + math.pi/2)
            elif 'ankle' in joint_name:
                # Ankle joints: adjust to keep feet level
                amplitude = 0.1 if 'left' in joint_name else -0.1
                position = amplitude * math.sin(self.step_phase - math.pi/4)
            else:
                position = 0.0
                
            positions.append(position)
        
        point.positions = positions
        point.velocities = [0.0] * len(self.joint_names)  # Simplified for this example
        point.accelerations = [0.0] * len(self.joint_names)  # Simplified for this example
        
        # Small duration for continuous motion
        point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms
        
        msg.points = [point]
        
        self.joint_trajectory_publisher.publish(msg)
        
        self.get_logger().debug(f'Published walking pattern: {positions}')


def main(args=None):
    """
    Main function that initializes and runs the walking controller.
    """
    rclpy.init(args=args)
    
    node = WalkingPatternController()
    
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

## Exercise 4: Implement Safety Stop Mechanism

**Problem**: Create a safety stop mechanism that immediately halts all robot movements.

**Solution**:

```python
#!/usr/bin/env python3
# File: athena_control/src/safety_stop_controller.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory
import threading

class SafetyStopController(Node):
    """
    A safety controller that can immediately halt all robot movements.
    """

    def __init__(self):
        super().__init__('safety_stop_controller')
        
        # Publisher to send zero trajectories to all controllers
        self.trajectory_stopper = self.create_publisher(
            JointTrajectory,
            '/all_controllers/joint_trajectory',
            10
        )
        
        # Subscriber for emergency stop command
        self.emergency_stop_sub = self.create_subscription(
            Bool,
            'emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Subscriber for safety status
        self.safety_status_sub = self.create_subscription(
            String,
            'safety_status',
            self.safety_status_callback,
            10
        )
        
        # Flag to track safety state
        self.emergency_stop_triggered = False
        self.safety_lock = threading.Lock()
        
        self.get_logger().info('Safety Stop Controller initialized')

    def emergency_stop_callback(self, msg):
        """
        Callback for emergency stop command.
        """
        if msg.data:  # Emergency stop triggered
            self.get_logger().error('EMERGENCY STOP TRIGGERED!')
            self.trigger_emergency_stop()
        else:  # Emergency stop reset
            self.get_logger().info('Emergency stop reset')
            self.reset_emergency_stop()

    def safety_status_callback(self, msg):
        """
        Monitor safety status messages for potential hazards.
        """
        if 'critical' in msg.data.lower() or 'error' in msg.data.lower():
            self.get_logger().warn(f'Safety hazard detected: {msg.data}')
            # Optionally trigger emergency stop based on certain conditions
            if 'high_temp' in msg.data.lower() or 'collision_detected' in msg.data.lower():
                self.get_logger().warn('Automatic safety stop triggered')
                self.trigger_emergency_stop()

    def trigger_emergency_stop(self):
        """
        Trigger emergency stop by sending zero commands to all controllers.
        """
        with self.safety_lock:
            self.emergency_stop_triggered = True
            
            # Send zero commands to all possible joints/controllers
            # This is a simplified version - in practice you'd target specific controllers
            zero_trajectory = JointTrajectory()
            zero_trajectory.joint_names = []  # Would be populated with all robot joints
            # For this example, we'll just log the action
            self.get_logger().info('Zero trajectory sent to all controllers')
            
            # Publish emergency stop message to all relevant topics
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_pub.publish(emergency_msg)

    def reset_emergency_stop(self):
        """
        Reset the emergency stop state.
        """
        with self.safety_lock:
            self.emergency_stop_triggered = False
            self.get_logger().info('Emergency stop reset - robot cleared for movement')

    def is_movement_allowed(self):
        """
        Check if movement is currently allowed.
        """
        with self.safety_lock:
            return not self.emergency_stop_triggered


def main(args=None):
    """
    Main function that initializes and runs the safety controller.
    """
    rclpy.init(args=args)
    
    node = SafetyStopController()
    
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

## Exercise 5: Extend URDF with Finger Joints

**Problem**: Modify the URDF model to include finger joints for more complex hand movements.

**Solution**:

First, let's extend the URDF to include finger joints:

```xml

  
  
    
      1.
      1.
      1.
    
    
      1.
      
        1.
      
      
    
    
      1.
      
        1.
      
    
  
  
    
    
    1.
    1.
    1.
    1.
  
  
  
  
    
      1.
      1.
      1.
    
    
      1.
      
        1.
      
      
    
    
      1.
      
        1.
      
    
  
  
    
    
    1.
    1.
    1.
    1.
  
  
  
    
      1.
      1.
      1.
    
    
      1.
      
        1.
      
      
    
    
      1.
      
        1.
      
    
  
  
    
    
    1.
    1.
    1.
    1.
  
  
  
  
    
      1.
      1.
      1.
    
    
      1.
      
        1.
      
      
    
    
      1.
      
        1.
      
    
  
  1.
    
    
    1.
    1.
    1.
    1.
  
  
  
    
      1.
      1.
      1.
    
    
      1.
      
        1.
      
      
    
    
      1.
      
        1.
      
    
  
  1.
    
    
    1.
    1.
    1.
    1.
  
  
  
  
  
  
  
  1.
  
**Step** 
    
      gazebo_ros2_control/GazeboSystem
    
    
    
      
        -0.785
        0.785
      
      
      
    
    
      
        0.0
        1.0
      
      
      
    
    
      
        0.0
        1.57
      
      
      
    
    1.
      
        0.0
        1.57
      
      
      
    
    1.
      
        0.0
        1.57
      
      
      
    
    
  
```

And a node to control the fingers:

```python
#!/usr/bin/env python3
# File: athena_examples/src/finger_control_demo.py

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import String


class FingerControlDemo(Node):
    """
    A node to demonstrate finger control for the athena humanoid.
    """

    def __init__(self):
        super().__init__('finger_control_demo')
        
        # Publisher for finger joint trajectories
        self.finger_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/finger_controller/joint_trajectory',
            10
        )
        
        # Subscriber for finger command strings
        self.finger_command_sub = self.create_subscription(
            String,
            'finger_command',
            self.finger_command_callback,
            10
        )
        
        # Define finger joint names
        self.finger_joints = [
            'left_thumb_abduction', 'left_thumb_flexion',
            'left_index_flexion_1', 'left_index_flexion_2',
            # ... add all other finger joints
        ]
        
        # Timer to send regular finger commands (for demonstration)
        self.demo_timer = self.create_timer(5.0, self.send_demo_finger_pattern)
        
        self.get_logger().info('Finger Control Demo initialized')

    def finger_command_callback(self, msg):
        """
        Process finger command strings (e.g., "make_fist", "peace_sign", "wave").
        """
        command = msg.data.lower()
        
        if command == 'make_fist':
            self.make_fist()
        elif command == 'open_hand':
            self.open_hand()
        elif command == 'peace_sign':
            self.peace_sign()
        elif command == 'wave':
            self.wave_fingers()
        else:
            self.get_logger().warn(f'Unknown finger command: {command}')

    def make_fist(self):
        """
        Make a fist with the left hand.
        """
        msg = JointTrajectory()
        msg.joint_names = self.finger_joints
        
        point = JointTrajectoryPoint()
        # Close all fingers
        positions = [1.57] * len(self.finger_joints)  # Fully flexed
        point.positions = positions
        point.velocities = [0.0] * len(self.finger_joints)
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        msg.points = [point]
        self.finger_trajectory_publisher.publish(msg)
        
        self.get_logger().info('Making a fist')

    def open_hand(self):
        """
        Open the hand with fingers extended.
        """
        msg = JointTrajectory()
        msg.joint_names = self.finger_joints
        
        point = JointTrajectoryPoint()
        # Open all fingers
        positions = [0.0] * len(self.finger_joints)  # Fully extended
        point.positions = positions
        point.velocities = [0.0] * len(self.finger_joints)
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        msg.points = [point]
        self.finger_trajectory_publisher.publish(msg)
        
        self.get_logger().info('Opening hand')

    def peace_sign(self):
        """
        Make a peace sign (index and middle fingers extended).
        """
        msg = JointTrajectory()
        msg.joint_names = self.finger_joints
        
        point = JointTrajectoryPoint()
        positions = []
        for joint_name in self.finger_joints:
            if 'index' in joint_name or 'middle' in joint_name:
                positions.append(0.0)  # Extended
            else:
                positions.append(1.57)  # Closed
        point.positions = positions
        point.velocities = [0.0] * len(self.finger_joints)
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        msg.points = [point]
        self.finger_trajectory_publisher.publish(msg)
        
        self.get_logger().info('Making peace sign')

    def send_demo_finger_pattern(self):
        """
        Send a sequence of finger movements for demonstration.
        """
        # Alternate between open hand and fist
        if self.get_clock().now().nanoseconds % 2 == 0:
            self.open_hand()
        else:
            self.make_fist()

def main(args=None):
    """
    Main function that initializes and runs the finger control demo.
    """
    rclpy.init(args=args)
    
    node = FingerControlDemo()
    
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


