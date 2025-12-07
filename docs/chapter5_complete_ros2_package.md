---
sidebar_position: 6
title: Chapter 5 - Complete ROS 2 Package
---

# Chapter 5: Building Your First Complete ROS 2 Humanoid Package (with templates)

## Learning Objectives

By the end of this chapter, you should be able to:
- Create a complete ROS 2 workspace with all necessary packages for a humanoid robot
- Implement the athena_description package with URDF and mesh files
- Develop the athena_bringup package with appropriate launch files
- Configure the athena_control package with controllers for the humanoid
- Set up the athena_gazebo simulation environment
- Execute a JointTrajectory command that makes the robot wave
- Use colcon build and source commands for workspace management
- Apply best practices for organizing ROS 2 packages
- Create and validate launch files that start Gazebo + RViz2
- Understand how all previous concepts integrate into a cohesive system

## 5.1 Introduction: The Complete ROS 2 Workspace

This chapter brings together all the concepts learned in the previous chapters to create a complete, functional ROS 2 workspace for the "athena" humanoid robot. The workspace will include four main packages:

1. `athena_description`: Contains the URDF model and mesh files
2. `athena_bringup`: Contains launch files to start the complete system
3. `athena_control`: Contains controller configurations for the humanoid
4. `athena_gazebo`: Contains files necessary for simulating the "athena" humanoid in Gazebo

This complete package will allow you to launch Gazebo + RViz2 with the "athena" humanoid model standing, and execute a JointTrajectory command to make the robot wave.

## 5.2 Creating the Workspace Structure

Before implementing the individual packages, we need to establish the proper ROS 2 workspace structure:

```bash
# Create the workspace
mkdir -p ~/athena_ws/src
cd ~/athena_ws

# Create package directories
mkdir -p src/{athena_description,athena_bringup,athena_control,athena_gazebo}
```

We'll also create a custom message package for AI commands:

```bash
cd ~/athena_ws/src
ros2 pkg create --name athena_interfaces --dependencies std_msgs builtin_interfaces --maintainer-email="your-email@example.com" --maintainer-name="Your Name" --description="Custom interfaces for the Athena humanoid" --license "Apache-2.0"
```

## 5.3 Implementing athena_description Package

The `athena_description` package contains the URDF model and mesh files for the "athena" humanoid. We'll expand on the URDF model created in Chapter 4, adding the necessary files for controllers and simulation:

### 5.3.1 Package Structure

```
athena_description/
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ CMakeLists.txt
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ package.xml
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ urdf/
Ã¢â€â€š   Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ athena.urdf
Ã¢â€â€š   Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ athena_fixed.urdf
Ã¢â€â€š   Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ athena_floating.urdf
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ meshes/
Ã¢â€â€š   Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ base_link.dae
Ã¢â€â€š   Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ head.dae
Ã¢â€â€š   Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ left_shoulder.dae
Ã¢â€â€š   Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ [more mesh files...]
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ launch/
Ã¢â€â€š   Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ view_athena.launch.py
Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ config/
    Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ joint_limits.yaml
```

### 5.3.2 CMakeLists.txt and package.xml

```cmake
cmake_minimum_required(VERSION 3.8)
project(athena_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install URDF files
install(DIRECTORY
  urdf
  meshes
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

```xml
1.
1.
1.
  athena_description
  0.1.0
  URDF models for the Athena humanoid robot
  Your Name
  Apache-2.0

  ament_cmake

  urdf
  xacro
  robot_state_publisher
  rviz2

  ament_lint_auto
  ament_lint_common

  
    ament_cmake
  

```

## 5.4 Implementing athena_bringup Package

The `athena_bringup` package contains launch files that start the complete system. This is where we tie together all components:

### 5.4.1 Package Structure

```
athena_bringup/
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ CMakeLists.txt
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ package.xml
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ launch/
Ã¢â€â€š   Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ athena_world.launch.py
Ã¢â€â€š   Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ athena_simulation.launch.py
Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ config/
    Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ startup_params.yaml
```

### 5.4.2 Launch File Implementation

Here's the main launch file that starts Gazebo + RViz2 with the humanoid model standing:

```python
# File: athena_bringup/launch/athena_world.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')

    # Set parameters
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless'
    )

    declare_headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Set to "true" for headless mode'
    )

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'verbose': 'false',
            'pause': 'false',
            'gui_required': gui,
            'headless': headless,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Robot State Publisher node
    robot_description_content = open(
        PathJoinSubstitution([
            FindPackageShare('athena_description'),
            'urdf',
            'athena.urdf'
        ]).perform({}), 'r').read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'athena',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Launch RViz
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('athena_description'),
        'rviz',
        'view_athena.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    # Joint State Publisher GUI (for testing purposes)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui)
    )

    # Static transform publisher to connect world to base_link
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link']
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_gui_arg,
        declare_headless_arg,
        declare_namespace_arg,
        SetParameter(name='use_sim_time', value=True),
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
        joint_state_publisher_gui,
        static_transform_publisher
    ])
```

## 5.5 Implementing athena_control Package

The `athena_control` package contains the controller configurations for the humanoid robot:

### 5.5.1 Package Structure

```
athena_control/
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ CMakeLists.txt
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ package.xml
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ config/
Ã¢â€â€š   Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ athena_controllers.yaml
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ launch/
Ã¢â€â€š   Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ athena_control.launch.py
Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ controllers/
    Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ athena_controller_manager.yaml
```

### 5.5.2 Controller Configuration File

```yaml
# File: athena_control/config/athena_controllers.yaml

controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

joint_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - left_shoulder_yaw
      - left_elbow_pitch
      - right_shoulder_yaw
      - right_elbow_pitch
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

## 5.6 Implementing athena_gazebo Package

The `athena_gazebo` package contains files necessary for simulating the "athena" humanoid in Gazebo:

### 5.6.1 Package Structure

```
athena_gazebo/
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ CMakeLists.txt
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ package.xml
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ launch/
Ã¢â€â€š   Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ athena_gazebo.launch.py
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ models/
Ã¢â€â€š   Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ athena/
Ã¢â€â€š       Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ model.sdf
Ã¢â€â€š       Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ meshes/
Ã¢â€Å“Ã¢â€â‚¬Ã¢â€â‚¬ worlds/
Ã¢â€â€š   Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ athena_world.world
Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ config/
    Ã¢â€â€Ã¢â€â‚¬Ã¢â€â‚¬ gazebo_params.yaml
```

### 5.6.2 Gazebo Launch File

```python
# File: athena_gazebo/launch/athena_gazebo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Gazebo server
    gzserver = Node(
        package='gazebo_ros',
        executable='gzserver',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[
            PathJoinSubstitution([
                FindPackageShare('athena_gazebo'),
                'worlds',
                'athena_world.world'
            ]),
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='both'
    )

    # Gazebo client
    gzclient = Node(
        package='gazebo_ros',
        executable='gzclient',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='both',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    return LaunchDescription([
        declare_use_sim_time_arg,
        gzserver,
        gzclient
    ])
```

## 5.7 Creating the Waving Motion Demo

Now let's create the Python script that makes the robot wave with a JointTrajectory command:

```python
#!/usr/bin/env python3
# File: athena_examples/src/chapter5_waving_demo.py

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class AthenaWavingDemo(Node):
    """
    Node to make the athena humanoid robot wave.
    """

    def __init__(self):
        super().__init__('athena_waving_demo')
        
        # Publisher for joint trajectories
        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Timer to send wave command periodically
        self.timer = self.create_timer(5.0, self.wave_callback)
        
        # Define joint names for the athena humanoid right arm
        self.arm_joints = [
            'right_shoulder_yaw',
            'right_elbow_pitch'
        ]

        self.get_logger().info('Athena Waving Demo Node initialized')

    def wave_callback(self):
        """
        Send the waving motion trajectory.
        """
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints
        
        # Create trajectory points for waving motion
        points = []
        
        # Point 1: Neutral position
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0]  # Shoulder and elbow at neutral
        point1.velocities = [0.0, 0.0]
        point1.accelerations = [0.0, 0.0]
        point1.time_from_start = Duration(sec=0, nanosec=0)
        points.append(point1)
        
        # Point 2: Raise right arm to wave position
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
        
        self.get_logger().info(f'Published waving trajectory with {len(points)} points for {len(self.arm_joints)} joints')

def main(args=None):
    """
    Main function that initializes the node and spins it.
    """
    rclpy.init(args=args)
    
    waving_demo = AthenaWavingDemo()
    
    try:
        rclpy.spin(waving_demo)
    except KeyboardInterrupt:
        pass
    finally:
        waving_demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5.8 Using colcon Build and Source Commands

Now that we have all the packages created, let's build the workspace:

```bash
# Navigate to the workspace
cd ~/athena_ws

# Source ROS 2 Iron
source /opt/ros/iron/setup.bash

# Build the workspace
colcon build --packages-select athena_description athena_bringup athena_control athena_gazebo athena_interfaces

# Source the built workspace
source install/setup.bash
```

To build all packages in the workspace:

```bash
cd ~/athena_ws
source /opt/ros/iron/setup.bash
colcon build
source install/setup.bash
```

## 5.9 Launching the Complete System

Once the workspace is built and sourced, you can launch the complete system:

```bash
# Launch the world with athena humanoid
ros2 launch athena_bringup athena_world.launch.py
```

To run the waving demo in another terminal:

```bash
# Make sure to source the workspace in any new terminal
cd ~/athena_ws
source install/setup.bash

# Run the waving demo
ros2 run athena_examples chapter5_waving_demo
```

## 5.10 Verification and Testing

To verify that the complete system is functioning properly:

1. **Check that all packages build successfully**:
   ```bash
   colcon build
   ```

2. **Verify the robot model loads correctly**:
   ```bash
   # Check the robot description
   ros2 param get /robot_state_publisher robot_description
   
   # View the robot in RViz
   ros2 launch athena_description view_athena.launch.py
   ```

3. **Test the controller interface**:
   ```bash
   # List available controllers
   ros2 run controller_manager spawner --list-controllers
   
   # Check available topics
   ros2 topic list | grep joint
   ```

4. **Validate the full simulation**:
   - Launch the complete system with the launch file
   - Verify the robot appears in both Gazebo and RViz
   - Check that joint states are being published
   - Run the waving demo and observe the robot motion

## 5.11 Pro Tips: Package Organization Best Practices

- **Modular Design**: Keep packages focused and modular to promote reusability and maintainability
- **Proper Dependencies**: Define package dependencies correctly in package.xml and CMakeLists.txt
- **Consistent Naming**: Use consistent naming conventions across all packages
- **Documentation**: Include README files in each package explaining its purpose and usage
- **Configuration Separation**: Separate configuration from code to allow easy customization
- **Launch File Organization**: Structure launch files in a hierarchy that matches use cases
- **Testing**: Include tests in each package to verify functionality

## 5.12 Summary

This chapter has brought together all the concepts learned in the previous chapters to create a complete ROS 2 workspace with all necessary packages for the "athena" humanoid robot. We've implemented:

1. `athena_description`: Contains the URDF model and mesh files for the "athena" humanoid
2. `athena_bringup`: Contains launch files to start the complete system
3. `athena_control`: Contains controller configurations for the humanoid
4. `athena_gazebo`: Contains files necessary for simulating the "athena" humanoid in Gazebo
5. `athena_interfaces`: Custom messages and services for AI integration

We've also created a demo that makes the robot wave using joint trajectory commands. The system can be launched with Gazebo + RViz2 with the "athena" humanoid model standing, and the waving demo can be executed to make the robot perform the waving motion.

This completes Module 1: The Robotic Nervous System, providing a comprehensive foundation for understanding how to bridge AI agents with physical robotic systems using ROS 2. The module covers everything from the theoretical differences between digital and embodied AI to the practical implementation of complete humanoid robotics systems.

## Exercises

1. Create a launch file that starts only the controllers without launching Gazebo or RViz2.
2. Modify the waving motion to make it more complex (e.g., add left-arm movements).
3. Create a controller configuration for a walking gait pattern.
4. Implement a safety stop mechanism that immediately halts all robot movements.
5. Extend the URDF model to include finger joints for more complex hand movements.

### Solutions to Exercises

[Detailed solutions would be provided in the exercises appendix]


