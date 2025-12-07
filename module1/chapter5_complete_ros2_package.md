# Chapter 5: Building Your First ROS 2 Humanoid Package (with templates)

## Learning Objectives

By the end of this chapter, you should be able to:
- Create a complete ROS 2 workspace with all necessary packages for a humanoid robot
- Implement the athena_description package with URDF and mesh files
- Develop the athena_bringup package with appropriate launch files
- Configure the athena_control package with controllers for the humanoid
- Set up the athena_gazebo simulation environment
- Execute a JointTrajectory command that makes the "athena" humanoid robot wave
- Use colcon build and source commands to compile and deploy the workspace
- Understand ROS 2 workspace organization for complex robotic projects

## 5.1 Introduction: The Complete ROS 2 Workspace

In the previous chapters, we've explored the fundamental concepts of ROS 2, learned how to bridge AI agents with robots using rclpy, and mastered URDF/Xacro for humanoid robots. Now, it's time to pull everything together by creating a complete ROS 2 workspace with all necessary packages for the "athena" humanoid robot.

This chapter will guide you through the creation of a full ROS 2 ecosystem consisting of four main packages:
- athena_description: Contains the URDF model and mesh files for the "athena" humanoid
- athena_bringup: Contains launch files to start the complete system
- athena_control: Contains controller configurations for the humanoid
- athena_gazebo: Contains files necessary for simulating the "athena" humanoid in Gazebo

This complete package will allow you to launch Gazebo + RViz2 with the "athena" humanoid model standing, and execute a JointTrajectory command to make the robot wave.

## 5.2 Creating the Workspace Structure

Before we begin implementing the individual packages, we need to create the proper ROS 2 workspace structure. The workspace will be organized as follows:

```
~/athena_ws/
├── src/
│   ├── athena_description/
│   ├── athena_bringup/
│   ├── athena_control/
│   ├── athena_gazebo/
│   └── athena_examples/
```

Let's create this structure:

```bash
# Create the workspace
mkdir -p ~/athena_ws/src
cd ~/athena_ws

# Create package directories
mkdir -p src/{athena_description,athena_bringup,athena_control,athena_gazebo,athena_examples}
```

Now, we'll create the package.xml files for each package using ros2 pkg create command:

```bash
# Create athena_description package
cd ~/athena_ws/src
ros2 pkg create --license Apache-2.0 --description "URDF models for the Athena humanoid robot" athena_description

# Create athena_bringup package
ros2 pkg create --license Apache-2.0 --dependencies athena_description athena_control athena_gazebo --description "Launch files to bring up the Athena humanoid system" athena_bringup

# Create athena_control package
ros2 pkg create --license Apache-2.0 --dependencies athena_description --description "Controllers for the Athena humanoid robot" athena_control

# Create athena_gazebo package
ros2 pkg create --license Apache-2.0 --dependencies athena_description --description "Gazebo simulation files for the Athena humanoid robot" athena_gazebo

# Create athena_examples package
ros2 pkg create --license Apache-2.0 --dependencies athena_description --description "Example code for the Athena humanoid robot" athena_examples
```

## 5.3 Implementing athena_description Package

The `athena_description` package contains the URDF model and mesh files for the "athena" humanoid. We'll add the URDF file we created in Chapter 4, along with the mesh files:

### 5.3.1 URDF Model

First, copy the `athena.urdf` file with proper ROS 2 control interface integration:

```xml
<?xml version="1.0"?>
<robot name="athena" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Import materials -->
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
  <material name="blue">
    <color rgba="0 0 1 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <material name="red">
    <color rgba="1 0 0 1"/>
  </material>
  <material name="green">
    <color rgba="0 1 0 1"/>
  </material>

  <!-- Fixed base link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>
  <joint name="neck_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 1.0"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_shoulder">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
  </link>
  <joint name="left_shoulder_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder"/>
    <origin xyz="0.15 0.0 0.4"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <link name="left_elbow">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
  </link>
  <joint name="left_elbow_pitch" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_elbow"/>
    <origin xyz="0.0 0.0 -0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Right Arm -->
  <link name="right_shoulder">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.1"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.05"/>
      </geometry>
    </collision>
  </link>
  <joint name="right_shoulder_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="right_shoulder"/>
    <origin xyz="-0.15 0.0 0.4"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <link name="right_elbow">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.04" ixy="0.0" ixz="0.0" iyy="0.04" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="green"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
  </link>
  <joint name="right_elbow_pitch" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_elbow"/>
    <origin xyz="0.0 0.0 -0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_hip">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.08"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.08"/>
      </geometry>
    </collision>
  </link>
  <joint name="left_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0.05 0.0 -0.5"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="left_knee">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.07"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.07"/>
      </geometry>
    </collision>
  </link>
  <joint name="left_knee_pitch" type="revolute">
    <parent link="left_hip"/>
    <child link="left_knee"/>
    <origin xyz="0.0 0.0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="200" velocity="1"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="left_foot">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.05 0 -0.02"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.04"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.04"/>
      </geometry>
    </collision>
  </link>
  <joint name="left_ankle_pitch" type="revolute">
    <parent link="left_knee"/>
    <child link="left_foot"/>
    <origin xyz="0.0 0.0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_hip">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.08"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.08"/>
      </geometry>
    </collision>
  </link>
  <joint name="right_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="-0.05 0.0 -0.5"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="200" velocity="1"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="right_knee">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.07"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.5" radius="0.07"/>
      </geometry>
    </collision>
  </link>
  <joint name="right_knee_pitch" type="revolute">
    <parent link="right_hip"/>
    <child link="right_knee"/>
    <origin xyz="0.0 0.0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="200" velocity="1"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="right_foot">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.05 0 -0.02"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.04"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.02" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.04"/>
      </geometry>
    </collision>
  </link>
  <joint name="right_ankle_pitch" type="revolute">
    <parent link="right_knee"/>
    <child link="right_foot"/>
    <origin xyz="0.0 0.0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- ros2_control interface -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="left_shoulder_yaw">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="left_elbow_pitch">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_shoulder_yaw">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_elbow_pitch">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="left_hip_yaw">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="left_knee_pitch">
      <command_interface name="position">
        <param name="min">0</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="left_ankle_pitch">
      <command_interface name="position">
        <param name="min">-0.5</param>
        <param name="max">0.5</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_hip_yaw">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_knee_pitch">
      <command_interface name="position">
        <param name="min">0</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_ankle_pitch">
      <command_interface name="position">
        <param name="min">-0.5</param>
        <param name="max">0.5</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>

</robot>
```

### 5.3.2 Mesh Files

The mesh files would go in the `meshes` directory. Since creating actual 3D mesh files is beyond the scope of this chapter, we'll outline the structure:

```
athena_description/
├── urdf/
│   └── athena.urdf
├── meshes/
│   ├── base_link.stl
│   ├── head.stl
│   ├── left_shoulder.stl
│   ├── left_elbow.stl
│   ├── right_shoulder.stl
│   ├── right_elbow.stl
│   ├── left_hip.stl
│   ├── left_knee.stl
│   ├── left_foot.stl
│   ├── right_hip.stl
│   ├── right_knee.stl
│   └── right_foot.stl
└── CMakeLists.txt
```

## 5.4 Implementing athena_bringup Package

The `athena_bringup` package contains launch files that start the complete system. Let's create the launch file that starts Gazebo + RViz2 with the "athena" humanoid standing:

```python
# File: athena_bringup/launch/athena_world.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package share directory
    athena_description_dir = get_package_share_directory('athena_description')
    athena_control_dir = get_package_share_directory('athena_control')
    athena_gazebo_dir = get_package_share_directory('athena_gazebo')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('gazebo_ros'),
            '/launch/gazebo.launch.py']),
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'athena'],
        output='screen'
    )

    # Launch robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[os.path.join(athena_description_dir, 'urdf', 'athena.urdf')]
    )

    # Launch joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['joint_states'],
            'rate': 50.0,
        }],
    )

    # Launch controllers
    controller_manager = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    position_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    # Launch RViz
    rviz_config = os.path.join(athena_description_dir, 'rviz', 'athena.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        robot_state_publisher,
        joint_state_publisher,
        controller_manager,
        position_controller,
        rviz
    ])
```

## 5.5 Implementing athena_control Package

The `athena_control` package contains controller configurations for the humanoid. Let's create the controller configuration file:

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

The `athena_gazebo` package contains files necessary for simulating the "athena" humanoid in Gazebo. Let's create the necessary files:

```xml
<!-- File: athena_gazebo/launch/athena_gazebo.launch.py -->
<launch>
  <!-- Start Gazebo with empty world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <!-- Spawn the robot -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" 
        args="-param robot_description -urdf -model athena -x 0 -y 0 -z 1.0" 
        respawn="false" output="screen"/>
</launch>
```

```xml
<!-- File: athena_gazebo/urdf/athena.gazebo.xacro -->
<gazebo>
  <plugin name="gz_ros2_control" filename="libgz_ros2_control_system.so">
    <parameters>$(find athena_control)/config/athena_controllers.yaml</parameters>
  </plugin>
</gazebo>

<!-- For each link with visual properties -->
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
</gazebo>

<gazebo reference="head">
  <material>Gazebo/White</material>
</gazebo>

<gazebo reference="left_shoulder">
  <material>Gazebo/Green</material>
</gazebo>

<gazebo reference="left_elbow">
  <material>Gazebo/Green</material>
</gazebo>

<gazebo reference="right_shoulder">
  <material>Gazebo/Green</material>
</gazebo>

<gazebo reference="right_elbow">
  <material>Gazebo/Green</material>
</gazebo>

<gazebo reference="left_hip">
  <material>Gazebo/Red</material>
</gazebo>

<gazebo reference="left_knee">
  <material>Gazebo/Red</material>
</gazebo>

<gazebo reference="left_foot">
  <material>Gazebo/Black</material>
</gazebo>

<gazebo reference="right_hip">
  <material>Gazebo/Red</material>
</gazebo>

<gazebo reference="right_knee">
  <material>Gazebo/Red</material>
</gazebo>

<gazebo reference="right_foot">
  <material>Gazebo/Black</material>
</gazebo>
```

## 5.7 Creating the Waving Demo

Let's create the Python script that makes the robot wave with a JointTrajectory command:

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
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Timer to send wave command periodically
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.wave_callback)
        
        # Define joint names for the robot
        self.joint_names = [
            'left_shoulder_yaw', 'left_elbow_pitch',
            'right_shoulder_yaw', 'right_elbow_pitch',
            'left_hip_yaw', 'left_knee_pitch', 'left_ankle_pitch',
            'right_hip_yaw', 'right_knee_pitch', 'right_ankle_pitch'
        ]

        self.get_logger().info('Athena Waving Demo Node initialized')

    def wave_callback(self):
        """
        Callback to send the waving motion trajectory.
        """
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # Create trajectory points for waving motion
        points = []
        
        # Point 1: Starting position (ready to wave)
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0] * len(self.joint_names)  # Default position
        # Specifically position the right arm to start the wave
        point1.positions[self.joint_names.index('right_shoulder_yaw')] = 0.5
        point1.positions[self.joint_names.index('right_elbow_pitch')] = 0.5
        point1.time_from_start = Duration(sec=1, nanosec=0)
        points.append(point1)
        
        # Point 2: Wave up
        point2 = JointTrajectoryPoint()
        point2.positions = [0.0] * len(self.joint_names)  # Default position
        # Position for wave up
        point2.positions[self.joint_names.index('right_shoulder_yaw')] = 0.3
        point2.positions[self.joint_names.index('right_elbow_pitch')] = 1.0
        point2.time_from_start = Duration(sec=2, nanosec=0)
        points.append(point2)
        
        # Point 3: Wave down (return to center)
        point3 = JointTrajectoryPoint()
        point3.positions = [0.0] * len(self.joint_names)  # Default position
        # Position for wave down
        point3.positions[self.joint_names.index('right_shoulder_yaw')] = 0.5
        point3.positions[self.joint_names.index('right_elbow_pitch')] = 0.2
        point3.time_from_start = Duration(sec=3, nanosec=0)
        points.append(point3)
        
        # Point 4: Back to neutral position
        point4 = JointTrajectoryPoint()
        point4.positions = [0.0] * len(self.joint_names)  # Default position
        # Back to starting position
        point4.positions[self.joint_names.index('right_shoulder_yaw')] = 0.0
        point4.positions[self.joint_names.index('right_elbow_pitch')] = 0.0
        point4.time_from_start = Duration(sec=4, nanosec=0)
        points.append(point4)
        
        msg.points = points
        self.joint_trajectory_publisher.publish(msg)
        
        self.get_logger().info('Published waving trajectory for Athena humanoid')

def main(args=None):
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

## 5.8 Using Colcon Build and Source Commands

Now that we have all the packages created, let's build the workspace:

```bash
# Navigate to the workspace
cd ~/athena_ws

# Source ROS 2 Iron
source /opt/ros/iron/setup.bash

# Build the workspace
colcon build --packages-select athena_description athena_bringup athena_control athena_gazebo athena_examples

# Source the built workspace
source install/setup.bash
```

Alternatively, to build all packages in the workspace:

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

## 5.10 Pro Tips for ROS 2 Workspace Organization

- **Modular Design**: Keep packages focused and modular to promote reusability and maintainability
- **Proper Dependencies**: Define package dependencies correctly in package.xml and CMakeLists.txt
- **Consistent Naming**: Use consistent naming conventions across all packages
- **Documentation**: Include README files in each package explaining its purpose and usage
- **Configuration Separation**: Separate configuration from code to allow easy customization
- **Launch File Organization**: Structure launch files in a hierarchy that matches use cases
- **Testing**: Include tests in each package to verify functionality

## 5.11 Summary

In this chapter, we've created a complete ROS 2 workspace with all necessary packages for a humanoid robot. The workspace consists of:

1. `athena_description`: Contains the URDF model and mesh files for the "athena" humanoid
2. `athena_bringup`: Contains launch files to start the complete system
3. `athena_control`: Contains controller configurations for the humanoid
4. `athena_gazebo`: Contains files necessary for simulating the "athena" humanoid in Gazebo
5. `athena_examples`: Contains example code demonstrating the use of the system

We've also created a demo that makes the robot wave using joint trajectory commands. The system can be launched with Gazebo + RViz2 with the "athena" humanoid model standing, and the waving demo can be executed to make the robot perform the waving motion.

This completes the implementation of Module 1: The Robotic Nervous System, providing a comprehensive foundation for understanding how to bridge AI agents with physical robotic systems using ROS 2.

## Exercises

1. Create a launch file that starts only the controllers without launching Gazebo or RViz2.
2. Modify the waving motion to make it more complex (e.g., add left-arm movements).
3. Create a controller configuration for a walking gait pattern.
4. Implement a safety stop mechanism that immediately halts all robot movements.
5. Extend the URDF model to include finger joints for more complex hand movements.

### Solutions to Exercises

[To be included in the exercises appendix]