---
id: 2
title: book-module1-ros2-spec
stage: spec
date: 2025-12-07
surface: agent
model: 
feature: 001-book-module1-ros2
branch: 001-book-module1-ros2
user: 
command: /sp.specify
labels: [book, robotics, ros2, ai]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-book-module1-ros2/spec.md
 - specs/001-book-module1-ros2/checklists/requirements.md
tests:
 - 
---

## Prompt

$ARGUMENTS

You are an expert technical author writing the definitive 2025 practitioner's book on Physical AI and Humanoid Robotics. Write Module 1: The Robotic Nervous System (Weeks 1–5) exactly as it will appear in the final published book. The module must contain exactly these five chapters with this structure and tone: Chapter 1: From Digital AI to Embodied Intelligence Chapter 2: ROS 2 Humble/Iron Deep Dive (Nodes, Topics, Services, Actions) Chapter 3: rclpy – Bridging Python AI Agents to Robots Chapter 4: URDF/Xacro Mastery for Humanoids Chapter 5: Building Your First ROS 2 Humanoid Package (with templates) Overall guidelines for the entire module: - Total length: ~25,000–28,000 words across the five chapters - Audience: advanced undergraduate / graduate students and professional engineers who already know Python and basic ML but are new to robotics - Tone: authoritative, clear, and hands-on; every concept must be accompanied by working code that the reader can run today on Ubuntu 22.04 + ROS 2 Iron - Every chapter must include: - Learning objectives at the start - Fully reproducible code snippets (tested on ROS 2 Iron, December 2025) - Diagrams and tables where helpful (describe them in detail for the illustrator) - “Pro Tips” sidebars with real-world advice - End-of-chapter exercises with solutions in an appendix - References to the exact official ROS 2 documentation version - Use the official ROS 2 style guide for terminology and code formatting - Use a complete, production-ready humanoid example throughout the module: a 23-DoF simplified Unitree G1 / generic biped named “athena” (provide the full URDF + meshes on the companion GitHub repo: github.com/yourname/physical-ai-book) Specific chapter-level requirements: Chapter 1 (≈4,000 words) - Explain Moravec's Paradox, embodiment, and why 2025 is the inflection point for humanoid robotics - Contrast digital vs physical AI with concrete examples (ChatGPT vs Figure 02 / Tesla Optimus) - End with a vision of a $700 Jetson kit controlling a real humanoid by the end of the book Chapter 2 (≈6,000 words) - Complete reference for core concepts: nodes, topics, services, actions, parameters, lifecycle nodes - Comparison table: ROS 1 vs ROS 2 Iron (DDS, security, real-time, SROS2) - Detailed message flow diagrams for a humanoid walking example Chapter 3 (≈5,000 words) - Step-by-step creation of a Python AI agent (using modern async rclpy) that publishes joint trajectories - Show how to wrap any Hugging Face transformer or OpenAI API call inside a ROS 2 node - Include latency measurements and best practices for running LLMs on the same machine as real-time control Chapter 4 (≈6,000 words) - Full URDF + Xacro tutorial using the “athena” humanoid - Inertial parameters, transmission tags, gazebo plugins, safety controller tags - Visual vs collision meshes explanation with performance numbers - Provide both a simple fixed-base version and the full floating-base 23-DoF version Chapter 5 (≈6,000 words) - Walk the reader through creating a complete ROS 2 workspace from scratch - Final package structure: athena_description/, athena_bringup/, athena_control/, athena_gazebo/ - Include launch files that start Gazebo + RViz2 with the full humanoid already standing - End with the reader publishing a single JointTrajectory message that makes the robot wave Companion assets to reference: - All code must be in github.com/yourname/physical-ai-book/tree/main/module1 - Provide exact colcon build and source commands - Include a Dockerfile that sets up the complete environment in <5 minutes Write in Markdown with proper headings, code blocks (```python and ```xml), and placeholders for figures (![Figure 1.1: ...](figures/ch01_moravec.png)). Use first-person plural sparingly; prefer direct second-person ("you") for tutorials. Begin writing now with Chapter 1 and continue sequentially until all five chapters of Module 1 are complete.

## Response snapshot

# Feature Specification: Book Module 1 - The Robotic Nervous System

**Feature Branch**: `001-book-module1-ros2`
**Created**: 2025-12-07
**Status**: Draft
**Input**: User description: "You are an expert technical author writing the definitive 2025 practitioner's book on Physical AI and Humanoid Robotics. Write Module 1: The Robotic Nervous System (Weeks 1–5) exactly as it will appear in the final published book. The module must contain exactly these five chapters with this structure and tone: Chapter 1: From Digital AI to Embodied Intelligence Chapter 2: ROS 2 Humble/Iron Deep Dive (Nodes, Topics, Services, Actions) Chapter 3: rclpy – Bridging Python AI Agents to Robots Chapter 4: URDF/Xacro Mastery for Humanoids Chapter 5: Building Your First ROS 2 Humanoid Package (with templates)"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Complete Chapter 1: From Digital AI to Embodied Intelligence (Priority: P1)

As an advanced undergraduate student or professional engineer new to robotics, I want to understand the fundamental differences between digital AI and embodied intelligence, so I can appreciate why physical interaction with the world is crucial for AI development.

**Why this priority**: This foundational knowledge is essential before diving into the technical aspects of ROS 2 and humanoid robotics. Understanding Moravec's Paradox and the 2025 inflection point provides the motivation for the entire module.

**Independent Test**: Learner can explain the core concepts of Moravec's Paradox and the distinction between digital and physical AI, and articulate why 2025 is an important year for humanoid robotics development.

**Acceptance Scenarios**:

1. **Given** a learner who understands basic AI concepts, **When** they complete Chapter 1, **Then** they can articulate the challenges of embodied intelligence versus digital AI
2. **Given** concepts of ChatGPT vs real-world robotics examples, **When** the learner compares them, **Then** they understand why physical embodiment is harder than digital tasks
3. **Given** information about current humanoid platforms like Figure 02 or Tesla Optimus, **When** learner reviews these examples, **Then** they can explain the significance of 2025 as an inflection point for humanoid development

---

### User Story 2 - Master ROS 2 Core Concepts (Priority: P2)

As a reader of the book, I want to thoroughly understand ROS 2 Humble/Iron concepts including Nodes, Topics, Services, and Actions, so I can build robust robotic systems using these communication patterns.

**Why this priority**: Understanding ROS 2's communication architecture is fundamental to all subsequent chapters, making it the technical foundation for the rest of the module.

**Independent Test**: Learner can create, run, and debug basic ROS 2 nodes that communicate via topics, services, and actions using ROS 2 Iron on Ubuntu 22.04.

**Acceptance Scenarios**:

1. **Given** a computer with Ubuntu 22.04 and ROS 2 Iron installed, **When** the learner follows the Chapter 2 examples, **Then** they create and run working nodes with different communication patterns
2. **Given** examples of nodes, topics, services, and actions, **When** learner implements them, **Then** they demonstrate understanding of when to use each communication pattern
3. **Given** the comparison between ROS 1 and ROS 2, **When** learner reviews the differences, **Then** they understand the advantages of the DDS-based architecture

---

### User Story 3 - Create Python AI Agents with rclpy (Priority: P2)

As a reader of the book, I want to learn how to use rclpy to create Python AI agents that can interface with robots, so I can bridge the gap between AI models and physical robotic actions.

**Why this priority**: This bridges the AI knowledge most readers already have with the robotics domain, making it essential for the AI-to-robotics transition.

**Independent Test**: Learner can implement Python nodes using rclpy that wrap AI models (like Hugging Face transformers or OpenAI API calls) and publish joint trajectories to control robots.

**Acceptance Scenarios**:

1. **Given** a basic understanding of Python and AI, **When** learner follows Chapter 3, **Then** they create a working AI agent using rclpy that publishes joint trajectories
2. **Given** examples of wrapping Hugging Face transformers in ROS 2 nodes, **When** learner implements one, **Then** they successfully integrate AI models into ROS 2
3. **Given** performance requirements for running LLMs on the same machine as real-time control, **When** learner optimizes their implementation, **Then** they achieve acceptable latency measurements

---

### User Story 4 - Master URDF and Xacro for Humanoid Robots (Priority: P3)

As a reader of the book, I want to gain proficiency with URDF and Xacro to describe humanoid robots, so I can create accurate and efficient robot models for simulation and control.

**Why this priority**: URDF is the standard for robot description in ROS ecosystem, so mastering it is essential for working with any robot, especially complex humanoids.

**Independent Test**: Learner can create and debug a complete URDF/Xacro model of the "athena" humanoid with 23-DoF, including proper inertial parameters, transmission tags, and Gazebo plugins.

**Acceptance Scenarios**:

1. **Given** the "athena" humanoid specifications, **When** learner creates a URDF model, **Then** it loads correctly in RViz and Gazebo
2. **Given** requirements for inertial parameters and transmission tags, **When** learner adds them to their URDF, **Then** the simulation behaves physically accurately
3. **Given** Xacro macros for common robot parts, **When** learner uses them, **Then** they create more efficient and maintainable robot descriptions

---

### User Story 5 - Build Complete ROS 2 Humanoid Package (Priority: P1)

As a reader of the book, I want to build a complete ROS 2 workspace with all necessary packages for a humanoid robot, so I can have a working foundation to build upon for more complex robotics projects.

**Why this priority**: This is the culmination of all previous chapters, demonstrating real-world application of all the concepts learned.

**Independent Test**: Learner can create a complete ROS 2 workspace with athena_description, athena_bringup, athena_control, and athena_gazebo packages that successfully launches Gazebo + RViz2 with the humanoid model standing, and can execute a JointTrajectory command to make the robot wave.

**Acceptance Scenarios**:

1. **Given** a clean Ubuntu 22.04 system, **When** learner follows Chapter 5 instructions, **Then** they create a complete ROS 2 workspace with all required packages
2. **Given** the package structure guidelines, **When** learner organizes their code accordingly, **Then** they can build and source the workspace successfully
3. **Given** the launch files, **When** learner runs them, **Then** Gazebo and RViz2 start with the humanoid robot model correctly positioned
4. **Given** a JointTrajectory message, **When** learner publishes it, **Then** the robot executes the waving motion as intended

### Edge Cases

- What happens when the reader does not have access to a powerful enough computer for simulation?
- How does the system handle different versions of ROS 2 Iron than those tested in December 2025?
- What if the companion GitHub repository is unavailable during study?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Module MUST contain exactly five chapters with the specified topics: From Digital AI to Embodied Intelligence, ROS 2 Humble/Iron Deep Dive, rclpy – Bridging Python AI Agents to Robots, URDF/Xacro Mastery for Humanoids, and Building Your First ROS 2 Humanoid Package
- **FR-002**: Content MUST target advanced undergraduate/graduate students and professional engineers with Python and basic ML knowledge but new to robotics
- **FR-003**: All code examples MUST be fully reproducible and tested on Ubuntu 22.04 + ROS 2 Iron as of December 2025
- **FR-004**: Each chapter MUST include learning objectives at the beginning
- **FR-005**: Each chapter MUST contain fully reproducible code snippets with proper syntax highlighting
- **FR-006**: Content MUST include detailed diagrams and tables where helpful for understanding concepts
- **FR-007**: Each chapter MUST feature "Pro Tips" sidebars with real-world advice
- **FR-008**: Each chapter MUST include end-of-chapter exercises with solutions in an appendix
- **FR-009**: Content MUST reference the exact official ROS 2 documentation version
- **FR-010**: Content MUST follow the official ROS 2 style guide for terminology and code formatting
- **FR-011**: Content MUST use a complete, production-ready humanoid example throughout: the 23-DoF simplified Unitree G1 / generic biped named "athena"
- **FR-012**: Full URDF and mesh files for "athena" humanoid MUST be provided on the companion GitHub repo: github.com/yourname/physical-ai-book
- **FR-013**: Chapter 1 content MUST explain Moravec's Paradox, embodiment, and why 2025 is the inflection point for humanoid robotics
- **FR-014**: Chapter 1 MUST contrast digital vs physical AI with concrete examples like ChatGPT vs Figure 02/Tesla Optimus
- **FR-015**: Chapter 2 MUST provide a complete reference for ROS 2 core concepts: nodes, topics, services, actions, parameters, lifecycle nodes
- **FR-016**: Chapter 2 MUST include a comparison table between ROS 1 and ROS 2 Iron (DDS, security, real-time, SROS2)
- **FR-017**: Chapter 3 MUST provide step-by-step instructions for creating a Python AI agent using rclpy that publishes joint trajectories
- **FR-018**: Chapter 3 MUST show how to wrap Hugging Face transformers or OpenAI API calls inside a ROS 2 node
- **FR-019**: Chapter 3 MUST include latency measurements and best practices for running LLMs on the same machine as real-time control
- **FR-020**: Chapter 4 MUST provide a full URDF + Xacro tutorial using the "athena" humanoid
- **FR-021**: Chapter 4 MUST cover inertial parameters, transmission tags, gazebo plugins, and safety controller tags
- **FR-022**: Chapter 4 MUST explain visual vs collision meshes with performance numbers
- **FR-023**: Chapter 4 MUST provide both a simple fixed-base version and the full floating-base 23-DoF version of the humanoid
- **FR-024**: Chapter 5 MUST walk the reader through creating a complete ROS 2 workspace from scratch
- **FR-025**: Chapter 5 MUST define the final package structure: athena_description/, athena_bringup/, athena_control/, athena_gazebo/
- **FR-026**: Chapter 5 MUST include launch files that start Gazebo + RViz2 with the full humanoid already standing
- **FR-027**: Chapter 5 MUST end with the reader publishing a single JointTrajectory message that makes the robot wave
- **FR-028**: All code MUST be available in github.com/yourname/physical-ai-book/tree/main/module1
- **FR-029**: Content MUST provide exact colcon build and source commands
- **FR-030**: Content MUST include a Dockerfile that sets up the complete environment in <5 minutes
- **FR-031**: Module MUST be written in Markdown with proper headings, code blocks (```python and ```xml), and placeholders for figures
- **FR-032**: Module length MUST be ~25,000–28,000 words across the five chapters
- **FR-033**: Content MUST use direct second-person ("you") for tutorials rather than first-person plural

### Key Entities *(include if feature involves data)*

- **Book Module 1**: The complete deliverable containing five chapters on the robotic nervous system, totaling 25,000-28,000 words
- **"Athena" Humanoid**: The 23-DoF simplified Unitree G1 / generic biped robot model used throughout the module as the primary example
- **ROS 2 (Humble/Iron)**: The Robot Operating System version used as the foundational framework for all examples and exercises
- **rclpy**: The Python client library for ROS 2 that enables Python-based AI agents to interface with the robot
- **URDF/Xacro**: The XML-based formats used for describing robot models, with Xacro providing macro functionality for more complex descriptions
- **Chapter Content**: Each of the five chapters with specific learning objectives, code examples, diagrams, and exercises
- **Code Examples**: All reproducible Python and XML code snippets that run on Ubuntu 22.04 with ROS 2 Iron
- **Simulation Environment**: The Gazebo + RViz2 setup for testing and visualizing the "athena" humanoid robot
- **Package Structure**: The ROS 2 workspace organization including athena_description, athena_bringup, athena_control, and athena_gazebo packages
- **Companion GitHub Repository**: The code repository at github.com/yourname/physical-ai-book containing all source code and assets
- **Docker Environment**: The containerized setup that allows users to quickly get started with the required tools

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Learners complete all five chapters with 85% comprehension of core concepts as measured by end-of-chapter exercises
- **SC-002**: All code examples compile and run successfully on Ubuntu 22.04 with ROS 2 Iron in 100% of test environments
- **SC-003**: Learners can independently create a complete ROS 2 workspace with all required packages after completing Chapter 5
- **SC-004**: Learners can execute a JointTrajectory command that makes the "athena" humanoid robot wave at the end of the module
- **SC-005**: The module content totals between 25,000 and 28,000 words across the five chapters
- **SC-006**: Learners can articulate why 2025 is an inflection point for humanoid robotics development
- **SC-007**: Learners demonstrate proficiency with ROS 2 communication patterns (nodes, topics, services, actions) through practical exercises
- **SC-008**: Learners can create and debug a complete URDF/Xacro model for the "athena" humanoid robot
- **SC-009**: Learners can wrap AI models (Hugging Face transformers or OpenAI API calls) inside ROS 2 nodes
- **SC-010**: The Dockerfile sets up the complete environment in less than 5 minutes 95% of the time
- **SC-011**: 90% of users successfully complete the full simulation workflow with Gazebo and RViz2
- **SC-012**: Users can implement the "athena" humanoid model with both fixed-base and floating-base configurations

## Outcome

- ✅ Impact: Created comprehensive feature specification for robotics book module
- 🧪 Tests: No tests needed for specification creation
- 📁 Files: 2 files created (spec.md, requirements.md)
- 🔁 Next prompts: Ready for /sp.plan or /sp.clarify
- 🧠 Reflection: Successfully captured all requirements for the 5-chapter book module with specific technical focus

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A