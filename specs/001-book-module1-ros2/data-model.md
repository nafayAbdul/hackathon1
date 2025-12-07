# Data Model: Book Module 1 - The Robotic Nervous System

## Overview
This document describes the data models and entities relevant to Module 1: The Robotic Nervous System of the Physical AI and Humanoid Robotics book. These entities represent the key concepts and structures that will be taught throughout the module.

## Entity 1: Book Module 1
- **Description**: The complete deliverable containing five chapters on the robotic nervous system, totaling 25,000-28,000 words
- **Attributes**:
  - id: string (unique identifier for the module)
  - title: string ("The Robotic Nervous System")
  - word_count: integer (between 25,000 and 28,000)
  - chapters: array of Chapter objects
  - target_audience: string ("Advanced undergraduate/graduate students and professional engineers with Python and basic ML knowledge but new to robotics")
  - created_date: date
  - last_updated: date

## Entity 2: Chapter
- **Description**: Each of the five chapters with specific learning objectives, code examples, diagrams, and exercises
- **Attributes**:
  - id: string (unique identifier for the chapter)
  - module_id: string (reference to parent Book Module 1)
  - title: string (e.g., "From Digital AI to Embodied Intelligence")
  - word_count: integer (4,000-6,000 words)
  - learning_objectives: array of strings
  - code_examples: array of CodeExample objects
  - diagrams: array of Diagram objects
  - exercises: array of Exercise objects
  - pro_tips: array of strings

## Entity 3: Code Example
- **Description**: All reproducible Python and XML code snippets that run on Ubuntu 22.04 with ROS 2 Iron
- **Attributes**:
  - id: string (unique identifier for the code example)
  - chapter_id: string (reference to parent Chapter)
  - language: string ("Python", "XML", "Bash", etc.)
  - code_snippet: text (the actual code)
  - description: string (what the code does)
  - filename: string (how the file should be named)
  - dependencies: array of strings (what other code/packages this depends on)
  - tested_on: string ("Ubuntu 22.04 + ROS 2 Iron")
  - latency_target: float (in milliseconds, <100ms)

## Entity 4: "Athena" Humanoid
- **Description**: The 23-DoF simplified Unitree G1 / generic biped robot model used throughout the module as the primary example
- **Attributes**:
  - id: string ("athena")
  - name: string ("Athena Humanoid")
  - degrees_of_freedom: integer (23)
  - base_type: string ("fixed-base" or "floating-base")
  - urdf_file: string (path to URDF file)
  - mesh_files: array of strings (paths to mesh files)
  - inertial_parameters: object (mass, center of mass, inertia tensor for each link)
  - transmission_tags: array of objects (describing how actuators connect to joints)
  - gazebo_plugins: array of objects (simulation-specific plugins)
  - safety_controller_tags: array of objects (safety constraints)

## Entity 5: ROS 2 Component
- **Description**: Core ROS 2 concepts including Nodes, Topics, Services, and Actions
- **Attributes**:
  - id: string (unique identifier for the component)
  - type: string ("Node", "Topic", "Service", "Action", "Parameter", "Lifecycle Node")
  - name: string (the actual ROS 2 name)
  - chapter_id: string (reference to the chapter where it's introduced)
  - description: string (what this component does)
  - usage_example: string (code snippet showing usage)
  - message_type: string (for Topics, Services, Actions)

## Entity 6: rclpy Component
- **Description**: Python client library components for ROS 2 that enable Python-based AI agents to interface with the robot
- **Attributes**:
  - id: string (unique identifier for the rclpy component)
  - name: string (e.g., "Node", "Publisher", "Subscriber", "Client", "Service")
  - function: string (what this component does)
  - chapter_id: string (reference to the chapter where it's introduced)
  - usage_example: string (code snippet showing usage)
  - parameters: array of objects (configuration parameters)

## Entity 7: URDF/Xacro Element
- **Description**: XML-based elements used for describing robot models, with Xacro providing macro functionality
- **Attributes**:
  - id: string (unique identifier for the element)
  - type: string ("URDF" or "Xacro")
  - name: string (the name of the element)
  - parent_link: string (for joints, the parent link name)
  - child_link: string (for joints, the child link name)
  - joint_type: string ("revolute", "prismatic", "fixed", etc.)
  - origin_xyz: array of floats [x, y, z] (position relative to parent)
  - origin_rpy: array of floats [roll, pitch, yaw] (rotation relative to parent)
  - visual_mesh: string (path to visual mesh file)
  - collision_mesh: string (path to collision mesh file)
  - inertial_mass: float (mass of the link)
  - inertial_inertia: object (inertia tensor)

## Entity 8: Exercise
- **Description**: End-of-chapter exercises with solutions for student assessment
- **Attributes**:
  - id: string (unique identifier for the exercise)
  - chapter_id: string (reference to parent Chapter)
  - title: string (brief description of the exercise)
  - description: text (detailed instructions)
  - difficulty: string ("beginner", "intermediate", "advanced")
  - type: string ("theoretical", "hands-on", "simulation")
  - solution: text (the solution to the exercise)
  - solution_path: string (path to solution file if separate)

## Entity 9: Diagram
- **Description**: Detailed diagrams and tables where helpful for understanding concepts
- **Attributes**:
  - id: string (unique identifier for the diagram)
  - chapter_id: string (reference to parent Chapter)
  - title: string (brief description of the diagram)
  - type: string ("flowchart", "architecture", "sequence", "table", "illustration")
  - description: text (what the diagram shows)
  - file_path: string (path to the image file)
  - alt_text: string (accessibility text for the image)
  - caption: string (text that appears below the diagram)

## Entity 10: Pro Tip
- **Description**: "Pro Tips" sidebars with real-world advice mentioned in the requirements
- **Attributes**:
  - id: string (unique identifier for the tip)
  - chapter_id: string (reference to parent Chapter)
  - title: string (brief title for the tip)
  - content: text (the actual tip/advice)
  - category: string ("performance", "security", "best-practice", "troubleshooting", "real-world")

## Entity 11: Simulation Environment
- **Description**: The Gazebo + RViz2 setup for testing and visualizing the "athena" humanoid robot
- **Attributes**:
  - id: string (unique identifier for the environment)
  - name: string ("Gazebo + RViz2 Simulation Environment")
  - components: array of strings ("Gazebo", "RViz2", "Robot State Publisher", "Joint State Publisher")
  - launch_file: string (path to the launch file)
  - robot_model: string (reference to "athena" humanoid model)
  - plugins: array of strings (simulation-specific plugins)
  - controllers: array of strings (controller configurations)

## Entity 12: Package Structure
- **Description**: The ROS 2 workspace organization including athena_description, athena_bringup, athena_control, and athena_gazebo packages
- **Attributes**:
  - id: string (unique identifier for the package structure)
  - name: string (e.g., "athena_description", "athena_bringup", "athena_control", "athena_gazebo")
  - type: string ("description", "bringup", "control", "simulation")
  - contents: array of strings (file paths within the package)
  - dependencies: array of strings (other packages this package depends on)
  - launch_files: array of strings (paths to launch files in this package)
  - config_files: array of strings (paths to configuration files)