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
      $(find athena_description)/config/athena_control.yaml
    
  

  
  
    Gazebo/Blue
  

  
    Gazebo/White
  

  
    Gazebo/Grey
  

  
    Gazebo/Grey
  

  
    Gazebo/Grey
  

  
    Gazebo/Grey
  

  
    Gazebo/Grey
  

  
    Gazebo/Grey
  

  
    Gazebo/Grey
  

  
    Gazebo/Grey
  

  
    Gazebo/Grey
  

  
    Gazebo/Grey
  


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
- Ixx = 1/12 * m * (dÃ‚Â² + hÃ‚Â²)
- Iyy = 1/12 * m * (wÃ‚Â² + hÃ‚Â²)
- Izz = 1/12 * m * (wÃ‚Â² + dÃ‚Â²)

**Cylinder with mass m, radius r, height h**:
- Ixx = 1/12 * m * (3*rÃ‚Â² + hÃ‚Â²)
- Iyy = 1/12 * m * (3*rÃ‚Â² + hÃ‚Â²)
- Izz = 1/2 * m * rÃ‚Â²

**Sphere with mass m, radius r**:
- Ixx = Iyy = Izz = 2/5 * m * rÃ‚Â²

## 4.5 Transmission Tags

Transmission tags define how actuators (motors) connect to joints. In ROS 2, this is typically used with ros2_control for hardware interfaces.

Here's an example of transmission tags:

```xml


  transmission_interface/SimpleTransmission
  
    position_controllers/JointPositionController
  
  
    1
  




  transmission_interface/SimpleTransmission
  
    position_controllers/JointPositionController
  
  
    1
  

```

## 4.6 Gazebo Plugins and Simulation Considerations

For proper simulation in Gazebo, you need to define plugins that handle aspects like physics, sensors, and control.

### 4.6.1 ros2_control Plugin

The `gazebo_ros2_control` plugin connects the Gazebo simulation to the ros2_control framework:

```xml

  1.
    $(find athena_description)/config/athena_control.yaml
  

```

### 4.6.2 Joint Limits and Safety

When designing robots, it's important to define safety constraints:

```xml

  
  
  1.
  1.
  1.
  1.
  1.

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


```

## 4.9 Fixed-Base vs Floating-Base Configurations

For humanoid robotics, it's important to provide both fixed-base and floating-base configurations depending on the use case:

### 4.9.1 Fixed-Base Configuration

In the fixed-base configuration, the robot is anchored to the world frame, which is useful for:
- Testing manipulation tasks
- Stable control algorithm development
- Reduced computational requirements

```xml

1.


  
  
  
    
    
    1.
  

  
  


```

### 4.9.2 Floating-Base Configuration

In the floating-base configuration, the robot can move freely in space, which is essential for:
- Whole-body motion planning
- Walking and locomotion
- Simulating free-space behavior

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


