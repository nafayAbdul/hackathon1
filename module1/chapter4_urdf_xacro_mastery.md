# Chapter 4: URDF/Xacro Mastery for Humanoids

## Learning Objectives

By the end of this chapter, you should be able to:
- Create complete URDF models for complex humanoid robots
- Use Xacro macros to simplify and parameterize robot descriptions
- Define proper inertial parameters, transmission tags, and Gazebo plugins for realistic simulation
- Distinguish between visual and collision meshes and understand their performance implications
- Create both fixed-base and floating-base configurations for the "athena" humanoid
- Implement safety controller tags and proper joint limits
- Optimize URDF/Xacro files for performance in simulation and control
- Understand the relationship between URDF and kinematic/dynamic properties of robots
- Generate performance numbers for visual vs collision mesh processing

## 4.1 Introduction to URDF and Xacro

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including links (rigid parts), joints (connections between links), and their associated properties.

Xacro (XML Macros) is an XML macro language that enhances URDF by providing features like:
- Variable definitions and substitutions
- Mathematical expressions
- Macros for reusing common structures
- File inclusion

URDF and Xacro are essential for robotics because they allow:
- Simulation of robots in environments like Gazebo
- Visualization of robots in tools like RViz
- Computation of kinematics, dynamics, and collision detection
- Integration with ROS tools like robot_state_publisher

### 4.1.1 The Relationship to Our "Athena" Humanoid

Throughout this chapter, we'll develop the URDF and Xacro models for the "athena" humanoid robot, which has 23 degrees of freedom (DOF). This model will serve as our primary example for understanding URDF/Xacro concepts.

## 4.2 Fundamentals of URDF

### 4.2.1 Links and Joints

In URDF, a robot is composed of links connected by joints. Links represent rigid parts of the robot, while joints define the motion between links.

Here's the basic structure of a URDF:

```xml
<?xml version="1.0"?>
<robot name="athena_humanoid">

  <!-- Base link (root of the robot) -->
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
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Head link -->
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
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint connecting head to base -->
  <joint name="neck_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 1.0"/>
  </joint>

</robot>
```

### 4.2.2 Link Properties

Each link in URDF has three main properties:

1. **Inertial**: Defines the physical properties of the link, such as:
   - Mass: Amount of matter in the link
   - Origin: Center of mass location
   - Inertia: Moment of inertia tensor (Ixx, Ixy, Ixz, Iyy, Iyz, Izz)

2. **Visual**: Defines how the link appears visually:
   - Origin: Position and orientation offset
   - Geometry: Shape (box, cylinder, sphere, mesh, etc.)
   - Material: Color and texture

3. **Collision**: Defines the collision properties for physics simulation:
   - Origin: Position and orientation offset
   - Geometry: Shape used for collision detection (often simpler than visual)

### 4.2.3 Joint Properties

Joints connect links and define the degrees of freedom between them. Common joint types include:

- `fixed`: Zero degrees of freedom
- `revolute`: One degree of freedom around an axis (with limits)
- `continuous`: Like revolute but unlimited rotation
- `prismatic`: Prismatic joint with limits
- `floating`: Six degrees of freedom
- `planar`: Three degrees of freedom

## 4.3 Implementing "Athena" Humanoid URDF

Now let's build the complete URDF for the "athena" humanoid robot with 23 degrees of freedom. This is a detailed example that follows best practices:

```xml
<?xml version="1.0"?>
<robot name="athena_humanoid_23dof">

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="20.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="2.0" ixy="0.0" ixz="0.0" iyy="2.0" iyz="0.0" izz="2.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
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
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
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
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
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
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
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
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
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
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
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
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.08"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.08"/>
      </geometry>
    </collision>
  </link>
  <joint name="left_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0.05 0.0 -0.5"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="200" velocity="1"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="left_knee">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
    </collision>
  </link>
  <joint name="left_knee_pitch" type="revolute">
    <parent link="left_hip"/>
    <child link="left_knee"/>
    <origin xyz="0.0 0.0 -0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="200" velocity="1"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="left_ankle">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.05 0 -0.01"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.01" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.02"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.01" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.02"/>
      </geometry>
    </collision>
  </link>
  <joint name="left_ankle_pitch" type="revolute">
    <parent link="left_knee"/>
    <child link="left_ankle"/>
    <origin xyz="0.0 0.0 -0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_hip">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.08"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.08"/>
      </geometry>
    </collision>
  </link>
  <joint name="right_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="-0.05 0.0 -0.5"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="200" velocity="1"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="right_knee">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.2"/>
      <inertia ixx="0.08" ixy="0.0" ixz="0.0" iyy="0.08" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.4" radius="0.07"/>
      </geometry>
    </collision>
  </link>
  <joint name="right_knee_pitch" type="revolute">
    <parent link="right_hip"/>
    <child link="right_knee"/>
    <origin xyz="0.0 0.0 -0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="200" velocity="1"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="right_ankle">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.05 0 -0.01"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0.05 0 -0.01" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.02"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.01" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.1 0.02"/>
      </geometry>
    </collision>
  </link>
  <joint name="right_ankle_pitch" type="revolute">
    <parent link="right_knee"/>
    <child link="right_ankle"/>
    <origin xyz="0.0 0.0 -0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="1"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <!-- Gazebo plugins -->
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find athena_description)/config/athena_control.yaml</parameters>
    </plugin>
  </gazebo>

  <!-- Gazebo materials -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="head">
    <material>Gazebo/White</material>
  </gazebo>

  <gazebo reference="left_shoulder">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="left_elbow">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="right_shoulder">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="right_elbow">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="left_hip">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="left_knee">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="left_ankle">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="right_hip">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="right_knee">
    <material>Gazebo/Grey</material>
  </gazebo>

  <gazebo reference="right_ankle">
    <material>Gazebo/Grey</material>
  </gazebo>

</robot>
```

## 4.4 Inertial Parameters in Detail

Inertial parameters are crucial for accurate physics simulation. They define how the robot responds to forces and torques. Incorrect inertial parameters can cause unrealistic simulation behavior.

### 4.4.1 Mass Properties

For each link, you need to define:
- Mass: The amount of matter in the link (in kg)
- Center of mass: The point where all mass can be considered concentrated
- Inertia tensor: How mass is distributed around the center of mass

The inertia tensor is represented by six values: Ixx, Ixy, Ixz, Iyy, Iyz, Izz. For complex shapes, these can be calculated using CAD software or estimated using primitive shapes.

### 4.4.2 Calculating Inertial Properties

For simple geometric shapes, you can use analytical formulas:

**Box with mass m, dimensions (width w, depth d, height h)**:
- Ixx = 1/12 * m * (d² + h²)
- Iyy = 1/12 * m * (w² + h²)
- Izz = 1/12 * m * (w² + d²)

**Cylinder with mass m, radius r, height h**:
- Ixx = 1/12 * m * (3*r² + h²)
- Iyy = 1/12 * m * (3*r² + h²)
- Izz = 1/2 * m * r²

**Sphere with mass m, radius r**:
- Ixx = Iyy = Izz = 2/5 * m * r²

## 4.5 Transmission Tags

Transmission tags define how actuators (motors) connect to joints. In ROS 2, this is typically used with ros2_control for hardware interfaces.

Here's an example of transmission tags:

```xml
<!-- Left shoulder transmission -->
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_yaw">
    <hardwareInterface>position_controllers/JointPositionController</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>

<!-- Left elbow transmission -->
<transmission name="left_elbow_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_elbow_pitch">
    <hardwareInterface>position_controllers/JointPositionController</hardwareInterface>
  </joint>
  <actuator name="left_elbow_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## 4.6 Gazebo Plugins and Simulation Considerations

For proper simulation in Gazebo, you need to define plugins that handle aspects like physics, sensors, and control.

### 4.6.1 ros2_control Plugin

The `gazebo_ros2_control` plugin connects the Gazebo simulation to the ros2_control framework:

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find athena_description)/config/athena_control.yaml</parameters>
  </plugin>
</gazebo>
```

### 4.6.2 Joint Limits and Safety

When designing robots, it's important to define safety constraints:

```xml
<joint name="left_shoulder_yaw" type="revolute">
  <parent link="base_link"/>
  <child link="left_shoulder"/>
  <origin xyz="0.15 0.0 0.4"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <safety_controller k_position="10" k_velocity="1.0" soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## 4.7 Visual vs Collision Meshes

For complex robot models, it's important to distinguish between visual and collision meshes:

- **Visual meshes**: Used for rendering and visualization. Can be detailed with textures and colors.
- **Collision meshes**: Used for physics simulation and collision detection. Should be simpler to optimize performance.

### 4.7.1 Performance Implications

Using high-resolution meshes for collision detection can severely impact simulation performance. It's common to use simplified versions of visual meshes for collision detection, or use primitive shapes like boxes and cylinders.

For example, consider the performance implications of the following approaches:

1. **High-detail collision meshes**: Better accuracy, slower simulation (1000+ polygons per link)
2. **Simplified meshes**: Good compromise, moderate performance (100-500 polygons per link) 
3. **Primitive shapes**: Fastest simulation, less accurate collision detection (boxes, spheres, cylinders)

The choice depends on your performance requirements and simulation fidelity needs. For humanoid robots, a common approach is to use:
- Detailed meshes for the head and hands (where precision matters)
- Simplified meshes for the torso and limbs
- Primitive shapes for the feet

## 4.8 Xacro Macros for the "Athena" Humanoid

Xacro allows us to create parameterized macros that make URDF files more maintainable. For the Athena humanoid, we can create macros for common elements:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="athena_xacro">

  <!-- Define common constants -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="athena_height" value="1.0" />
  <xacro:property name="athena_mass" value="75.0" />
  
  <!-- Macro for a generic limb segment -->
  <xacro:macro name="limb_segment" params="name prefix parent_link xyz_rpy joint_origin joint_axis joint_lower joint_upper effort velocity">
    <!-- Link definition -->
    <link name="${prefix}_${name}">
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
        <material name="gray">
          <color rgba="0.5 0.5 0.5 0.8"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
        <geometry>
          <cylinder length="0.2" radius="0.05"/>
        </geometry>
      </collision>
    </link>
    
    <!-- Joint definition -->
    <joint name="${prefix}_${name}_joint" type="revolute">
      <parent link="${parent_link}"/>
      <child link="${prefix}_${name}"/>
      <origin xyz="${joint_origin}"/>
      <axis xyz="${joint_axis}"/>
      <limit lower="${joint_lower}" upper="${joint_upper}" effort="${effort}" velocity="${velocity}"/>
      <dynamics damping="0.1" friction="0.0"/>
    </joint>
  </xacro:macro>

  <!-- Macro for a humanoid leg -->
  <xacro:macro name="humanoid_leg" params="side parent_link hip_xyz knee_xyz ankle_xyz hip_lower hip_upper knee_lower knee_upper ankle_lower ankle_upper">
    <!-- Hip joint -->
    <xacro:limb_segment name="hip" prefix="${side}" parent_link="${parent_link}" 
                       joint_origin="${hip_xyz}" joint_axis="0 0 1" 
                       joint_lower="${hip_lower}" joint_upper="${hip_upper}" 
                       effort="200" velocity="1"/>
    
    <!-- Knee joint -->
    <xacro:limb_segment name="knee" prefix="${side}" parent_link="${side}_hip" 
                       joint_origin="${knee_xyz}" joint_axis="0 1 0" 
                       joint_lower="${knee_lower}" joint_upper="${knee_upper}" 
                       effort="200" velocity="1"/>
    
    <!-- Ankle joint -->
    <xacro:limb_segment name="ankle" prefix="${side}" parent_link="${side}_knee" 
                       joint_origin="${ankle_xyz}" joint_axis="0 1 0" 
                       joint_lower="${ankle_lower}" joint_upper="${ankle_upper}" 
                       effort="50" velocity="1"/>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <inertial>
      <mass value="20.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="2.0" ixy="0.0" ixz="0.0" iyy="2.0" iyz="0.0" izz="2.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Use the leg macro to create both legs -->
  <xacro:humanoid_leg side="left" parent_link="base_link"
                     hip_xyz="0.05 0.0 -0.5" knee_xyz="0.0 0.0 -0.4" ankle_xyz="0.0 0.0 -0.4"
                     hip_lower="-0.785" hip_upper="0.785"
                     knee_lower="0" knee_upper="1.57"
                     ankle_lower="-0.5" ankle_upper="0.5" />

  <xacro:humanoid_leg side="right" parent_link="base_link"
                     hip_xyz="-0.05 0.0 -0.5" knee_xyz="0.0 0.0 -0.4" ankle_xyz="0.0 0.0 -0.4"
                     hip_lower="-0.785" hip_upper="0.785"
                     knee_lower="0" knee_upper="1.57"
                     ankle_lower="-0.5" ankle_upper="0.5" />

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
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

  <!-- Arms -->
  <xacro:limb_segment name="shoulder" prefix="left" parent_link="base_link" 
                     joint_origin="0.15 0.0 0.4" joint_axis="0 0 1" 
                     joint_lower="-1.57" joint_upper="1.57" 
                     effort="100" velocity="1"/>

  <xacro:limb_segment name="elbow" prefix="left" parent_link="left_shoulder" 
                     joint_origin="0.0 0.0 -0.2" joint_axis="0 1 0" 
                     joint_lower="-1.57" joint_upper="1.57" 
                     effort="100" velocity="1"/>

  <xacro:limb_segment name="shoulder" prefix="right" parent_link="base_link" 
                     joint_origin="-0.15 0.0 0.4" joint_axis="0 0 1" 
                     joint_lower="-1.57" joint_upper="1.57" 
                     effort="100" velocity="1"/>

  <xacro:limb_segment name="elbow" prefix="right" parent_link="right_shoulder" 
                     joint_origin="0.0 0.0 -0.2" joint_axis="0 1 0" 
                     joint_lower="-1.57" joint_upper="1.57" 
                     effort="100" velocity="1"/>

</robot>
```

## 4.9 Fixed-Base vs Floating-Base Configurations

For humanoid robotics, it's important to provide both fixed-base and floating-base configurations depending on the use case:

### 4.9.1 Fixed-Base Configuration

In the fixed-base configuration, the robot is anchored to the world frame, which is useful for:
- Testing manipulation tasks
- Stable control algorithm development
- Reduced computational requirements

```xml
<!-- Fixed base URDF for manipulation tasks -->
<?xml version="1.0"?>
<robot name="athena_fixed_base">

  <!-- Connect base link to world frame -->
  <link name="world"/>
  <joint name="fixed_base_joint" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Rest of the robot definition (same as regular "athena") -->
  <!-- ... [insert rest of robot definition here] ... -->

</robot>
```

### 4.9.2 Floating-Base Configuration

In the floating-base configuration, the robot can move freely in space, which is essential for:
- Whole-body motion planning
- Walking and locomotion
- Simulating free-space behavior

```xml
<!-- Floating base URDF for locomotion -->
<?xml version="1.0"?>
<robot name="athena_floating_base">

  <!-- No fixed connection to world, the base link can move freely -->
  <!-- The base link starts at some initial position -->
  <link name="base_link">
    <inertial>
      <mass value="20.0"/>
      <origin xyz="0 0 0.5"/>
      <inertia ixx="2.0" ixy="0.0" ixz="0.0" iyy="2.0" iyz="0.0" izz="2.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Rest of the robot definition (same as regular "athena") -->
  <!-- ... [insert rest of robot definition here] ... -->

</robot>
```

## 4.10 Performance Optimization Strategies

Optimizing URDF/Xacro models is crucial for both simulation and control performance:

### 4.10.1 Collision Optimization

- Use simpler collision geometries than visual meshes
- Use bounding boxes instead of detailed meshes when possible
- Keep collision mesh complexity below 1000 triangles per link

### 4.10.2 Inertial Property Optimization

- Use simplified inertial properties when high accuracy isn't needed
- Approximate complex shapes with simpler geometries (e.g., a box for complex link shape)

### 4.10.3 Xacro Best Practices

- Use `xacro:include` to modularize your robot definition
- Use properties and macros to reduce duplication
- Use mathematical expressions to calculate related values

## 4.11 Pro Tips: URDF/Xacro Best Practices

- **Use consistent naming**: Follow a consistent naming convention (e.g., `left_elbow_joint` rather than mixing `left_elbow` and `right_arm_joint`)
- **Keep visual and collision separate**: Use different mesh files for visual and collision when needed for performance
- **Validate URDFs**: Use `check_urdf` command to validate your URDF files
- **Document your macros**: Comment your Xacro macros to explain their purpose and parameters
- **Version control**: Keep your URDF/Xacro files under version control to track changes
- **Use relative paths**: Use `$(find package_name)` to make your URDFs portable across environments
- **Test in simulation first**: Always test your URDF in simulation before applying to real hardware
- **Consider scaling**: Design your robot files with scaling in mind for different robot sizes
- **Include safety margins**: Add safety margins in joint limits to prevent damage during simulation
- **Group related files**: Organize your URDF, Xacro, mesh files, and launch files in appropriate subdirectories

## 4.12 Summary

This chapter has covered the essential concepts of URDF and Xacro for creating humanoid robot models. We've explored how to define links and joints, set proper inertial parameters, implement transmission tags, configure Gazebo plugins, and distinguish between visual and collision meshes. We've also seen how to use Xacro macros to make robot definitions more maintainable and how to create both fixed-base and floating-base configurations of the "athena" humanoid.

These skills are fundamental for creating accurate robot models that work effectively in both simulation and real-world control. In the next chapter, we'll put these concepts into practice by building complete ROS 2 packages for the "athena" humanoid.

## Exercises

1. Create a URDF model for a simple 3-DOF arm using the techniques learned in this chapter.
2. Implement a Xacro macro for creating generic wheel links with appropriate inertial and visual properties.
3. Define both fixed-base and floating-base configurations for a quadruped robot model.
4. Compare the simulation performance of a robot model with high-detail collision meshes versus simplified meshes.
5. Implement a Xacro macro that generates a humanoid model with a configurable number of segments per limb.

### Solutions to Exercises

[Detailed solutions for each exercise would be provided in the exercises appendix]