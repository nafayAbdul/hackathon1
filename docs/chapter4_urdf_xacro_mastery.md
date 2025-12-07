---
sidebar_position: 5
title: Chapter 4 - URDF and Xacro Mastery
---

# Chapter 4: URDF/Xacro Mastery for Humanoids

## Learning Objectives

By the end of this chapter, you should be able to:
- Create complete URDF models for complex humanoid robots with 23-DoF
- Use Xacro macros to simplify and parameterize robot descriptions
- Properly define inertial parameters, transmission tags, and Gazebo plugins
- Distinguish between visual and collision meshes and understand their performance implications
- Create both fixed-base and floating-base configurations of the "athena" humanoid
- Optimize URDF/Xacro files for simulation performance
- Implement safety controller tags and proper joint limits

## 4.1 Introduction to URDF and Xacro

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including links (rigid parts), joints (connections between links), and their associated properties.

While URDF is powerful, it can become verbose and repetitive for complex robots. This is where Xacro comes in. Xacro (XML Macros) is an XML macro language that extends URDF with features like:
- Variable definitions and substitutions
- Mathematical expressions
- Macros for reusing common structures
- File inclusion

Both URDF and Xacro are essential for robotics because they allow:
- Simulation of robots in environments like Gazebo
- Visualization of robots in tools like RViz
- Computation of kinematics, dynamics, and collision detection
- Integration with ROS tools like robot_state_publisher

### 4.1.1 The "Athena" Humanoid Model

Throughout this chapter, we'll develop the URDF and Xacro models for the "athena" humanoid robot, which has 23 degrees of freedom (DOF). This model will serve as our primary example for understanding URDF/Xacro concepts.

## 4.2 Fundamentals of URDF

### 4.2.1 Links and Joints

In URDF, a robot is composed of links connected by joints. Links represent rigid parts of the robot, while joints define the motion between links.

Here's the basic structure of a URDF file:

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

The three main elements for each link are:

1. **Inertial**: Defines the physical properties of the link, such as mass, center of mass, and inertia tensor
2. **Visual**: Defines how the link appears visually, including shape, origin, and material
3. **Collision**: Defines the collision properties for physics simulation, often using simpler shapes than visual

### 4.2.2 Joint Types

Joints in URDF define the degrees of freedom between links. Common joint types include:

- `fixed`: Zero degrees of freedom
- `revolute`: One degree of freedom around an axis (with joint limits)
- `continuous`: Like revolute but without limits
- `prismatic`: Prismatic joint with limits
- `floating`: Six degrees of freedom
- `planar`: Three degrees of freedom

## 4.3 Detailed URDF Elements

### 4.3.1 Inertial Properties

The inertial properties are crucial for physics simulation and robot control. They consist of:
- `mass`: Amount of matter in the link
- `origin`: Center of mass location relative to the link frame
- `inertia`: Inertia tensor (Ixx, Ixy, Ixz, Iyy, Iyz, Izz)

For simple geometric shapes, you can calculate the inertia values analytically:

- **Box**: Ixx = 1/12 * m * (hÃ‚Â² + dÃ‚Â²), Iyy = 1/12 * m * (wÃ‚Â² + hÃ‚Â²), Izz = 1/12 * m * (wÃ‚Â² + dÃ‚Â²)
- **Cylinder**: Ixx = 1/12 * m * (3*rÃ‚Â² + hÃ‚Â²), Iyy = 1/12 * m * (3*rÃ‚Â² + hÃ‚Â²), Izz = 1/2 * m * rÃ‚Â²
- **Sphere**: Ixx = Iyy = Izz = 2/5 * m * rÃ‚Â²

### 4.3.2 Transmission Tags

Transmission tags define how actuators (motors) connect to joints. In ROS 2, this is typically used with ros2_control for hardware interfaces:

```xml

  transmission_interface/SimpleTransmission
  
    position_controllers/JointPositionInterface
  
  
    1
  

```

### 4.3.3 Gazebo Plugins

Gazebo plugins extend the functionality of your robot in simulation:

```xml

  1.
    $(find athena_description)/config/athena_control.yaml
  

```

## 4.4 Xacro Macros for Complex Humanoid Models

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


```

## 4.5 The Complete "Athena" Humanoid URDF

Now let's look at a more complete example of the "Athena" humanoid URDF with 23 DOF:

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
  
**Step** 
    
      gazebo_ros2_control/GazeboSystem
    
    
      
      
      
    
    
      
      
      
    
    
      
      
      
    
    
      
      
      
    
    
      
      
      
    
    
      
      
      
    
    
      
      
      
    
    
      
      
      
    
    
      
      
      
    
    
      
      
      
    
  


```

## 4.6 Fixed-Base vs Floating-Base Configurations

For humanoid robotics, it's important to provide both fixed-base and floating-base configurations:

### 4.6.1 Fixed-Base Configuration

In the fixed-base configuration, the robot is constrained to remain at a fixed location, which is useful for:
- Testing manipulation tasks
- Developing stable control algorithms
- Reducing computational requirements

```xml

1.


  
  
  
    
    
    1.
  

  
  


```

### 4.6.2 Floating-Base Configuration

In the floating-base configuration, the robot's base link is free to move in space, which is essential for:
- Locomotion studies
- Whole-body motion planning
- Simulating free-space behavior

This configuration is already shown in the complete URDF example above.

## 4.7 Visual vs Collision Meshes and Performance Implications

For complex robot models, it's important to distinguish between visual and collision meshes:

- **Visual meshes**: Used for appearance; can be detailed with textures and colors
- **Collision meshes**: Used for physics simulation; should be simpler to optimize performance

### 4.7.1 Performance Considerations

The complexity of collision meshes directly impacts simulation performance:
- High-detail collision meshes (1000+ polygons per link) can slow simulation significantly
- Simplified collision meshes (100-500 polygons per link) provide a good balance
- Primitive shapes (boxes, cylinders, spheres) offer the best performance

For the "Athena" humanoid, you might use:
- Detailed meshes for the head and hands (where precision matters)
- Simplified meshes for the torso and limbs
- Primitive shapes for the feet

## 4.8 Pro Tips: URDF/Xacro Best Practices

- **Use consistent naming**: Follow a consistent naming convention (e.g., `left_shoulder_joint` rather than mixing `left_shoulder` and `right_arm_joint`)
- **Keep visual and collision separate**: Use different mesh files for visual and collision when needed for performance
- **Validate URDFs**: Use `check_urdf` command to validate your URDF files
- **Use Xacro macros**: Reduce duplication with parameterized macros
- **Document your macros**: Comment your Xacro macros to explain their purpose and parameters
- **Use relative paths**: Use `$(find package_name)` to make your URDFs portable across environments
- **Test in simulation first**: Always test your URDF in simulation before applying to real hardware
- **Consider scaling**: Design your robot files with scaling in mind for different robot sizes
- **Include safety margins**: Add safety margins in joint limits to prevent damage during simulation

## 4.9 Summary

This chapter has covered the essential concepts of URDF and Xacro for creating humanoid robot models. We've explored how to define links and joints, set proper inertial parameters, implement transmission tags, configure Gazebo plugins, and distinguish between visual and collision meshes.

We've also seen how to use Xacro macros to make robot definitions more maintainable and how to create both fixed-base and floating-base configurations of the "athena" humanoid. Finally, we discussed performance optimization strategies when designing robot models.

These skills are fundamental for creating accurate robot models that work effectively in both simulation and real-world control. In the next chapter, we'll put these concepts into practice by building complete ROS 2 packages for the "athena" humanoid.

## Exercises

1. Create a URDF model for a simple 3-DOF arm using the techniques learned in this chapter.
2. Implement a Xacro macro for creating generic wheel links with appropriate inertial and visual properties.
3. Define both fixed-base and floating-base configurations for a quadruped robot model.
4. Compare the simulation performance of a robot model with high-detail collision meshes versus simplified meshes.
5. Implement a Xacro macro that generates a humanoid model with a configurable number of segments per limb.

### Solutions to Exercises

[Detailed solutions would be provided in the exercises appendix]


