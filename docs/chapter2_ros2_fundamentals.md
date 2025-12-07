---
sidebar_position: 3
title: Chapter 2 - ROS 2 Fundamentals
---

# Chapter 2: ROS 2 Humble/Iron Deep Dive (Nodes, Topics, Services, Actions)

## Learning Objectives

By the end of this chapter, you should be able to:
- Create, run, and debug basic ROS 2 nodes that communicate via topics, services, and actions
- Understand the fundamental differences between ROS 1 and ROS 2 architectures
- Implement nodes that communicate using the publish-subscribe pattern (topics)
- Implement nodes that communicate using the request-response pattern (services)
- Implement nodes that handle long-running tasks with feedback (actions)
- Compare and contrast the communication patterns and know when to use each
- Implement best practices for multi-robot systems and security considerations
- Understand the DDS-based architecture of ROS 2 Iron

## 2.1 Introduction to ROS 2 Architecture

Robot Operating System 2 (ROS 2) represents a complete redesign of the popular robotics framework to address the limitations and requirements of modern robotics applications. Unlike ROS 1, which relied on a centralized master architecture, ROS 2 embraces a distributed architecture built on top of DDS (Data Distribution Service).

This chapter provides a comprehensive deep dive into the core communication patterns in ROS 2: nodes, topics, services, and actions. Understanding these components is crucial before diving into AI-agent integration (Chapter 3) and robot description (Chapter 4).

### 2.1.1 The Evolution from ROS 1 to ROS 2

ROS 1 served the robotics community well, but its architecture had several limitations:
- Single point of failure (the master)
- Limited support for multiple robots coordination
- Difficulty with networking across unreliable connections
- No built-in security or quality of service controls

ROS 2 addressed these issues with a distributed architecture that doesn't require a central master, enabling:
- Better multi-robot scenarios
- More robust networking
- Quality of service (QoS) controls
- Security features through SROS2

## 2.2 Nodes: The Basic Computing Units

In ROS 2, nodes are the fundamental computational units that perform robot-specific work. A node is essentially a process that performs computation. Nodes in ROS 2 are designed to be:
- Lightweight and fast to start
- Isolated from other nodes (crashes don't bring down the system)
- Easily configurable through parameters
- Able to perform specific functions

### 2.2.1 Creating a Node in Python

To create a node in Python using rclpy (ROS Client Library for Python), we need to:

1. Import the required modules
2. Create a class that inherits from rclpy.Node
3. Initialize the node in the constructor
4. Register any publishers, subscribers, services, or actions

Here's a basic template for a ROS 2 node:

```python
import rclpy
from rclpy.node import Node

class BasicNode(Node):
    def __init__(self):
        super().__init__('basic_node_name')
        self.get_logger().info('Basic node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = BasicNode()
    
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

### 2.2.2 Node Parameters

Nodes in ROS 2 can accept parameters that configure their behavior. Parameters are declared in the node and can be set at runtime via command line, launch files, or parameter files.

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters
        self.declare_parameter('param_name', 'default_value')
        
        # Get parameter value
        param_value = self.get_parameter('param_name').value
        self.get_logger().info(f'Parameter value: {param_value}')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    
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

## 2.3 Topics: Publish-Subscribe Communication

Topics in ROS 2 implement a publish-subscribe communication pattern. This is an asynchronous and decoupled way of sharing data between nodes. The publisher sends messages without knowing who (if anyone) will receive them, and the subscriber receives messages without knowing who (if anyone) sent them.

### 2.3.1 Quality of Service (QoS)

One significant difference between ROS 1 and ROS 2 is the introduction of Quality of Service (QoS) profiles. QoS allows fine-tuning of the communication behavior between publishers and subscribers.

Common QoS settings include:
- **History Policy**: How many samples to keep in the queue
- **Reliability Policy**: Whether to guarantee delivery
- **Durability Policy**: Whether to store messages for late-joining subscribers
- **Deadline**: How frequently data should be published

### 2.3.2 Creating Publishers and Subscribers

Here's an example of a publisher and subscriber:

Publisher code:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.get_clock().now()}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    
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

Subscriber code:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)  # QoS history depth
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    
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

## 2.4 Services: Request-Response Communication

Services implement a synchronous request-response communication pattern. A client sends a request to a service server, which processes the request and returns a response. This is similar to HTTP requests or RPC (Remote Procedure Call).

### 2.4.1 Creating Services and Clients

Service server code:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server = ServiceServer()
    
    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        pass
    finally:
        service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Service client code:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class ServiceClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = ServiceClient()

    future = client.send_request(1, 2)

    try:
        rclpy.spin_until_future_completed(client, future)
        response = future.result()
        if response is not None:
            client.get_logger().info(f'Result of add_two_ints: {response.sum}')
        else:
            client.get_logger().info('No response received')
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2.5 Actions: Goal-Feedback-Result Communication

Actions are designed for long-running tasks that require feedback and the ability to be preempted. They implement a goal-feedback-result communication pattern, which is ideal for tasks like navigation, where you want to know the progress toward reaching a goal.

### 2.5.1 Creating Actions

Action server code:
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return Fibonacci.Result()
                
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Publishing feedback: {feedback_msg.partial_sequence}')
        
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    fibonacci_action_server = FibonacciActionServer()
    
    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        fibonacci_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2.6 Comparison Table: ROS 1 vs ROS 2 Iron

| Aspect | ROS 1 | ROS 2 Iron |
|--------|-------|------------|
| Architecture | Centralized (master-based) | Distributed (DDS-based) |
| Communication Middleware | Custom TCP/UDP | DDS (Data Distribution Service) |
| Multi-Robot Support | Limited, difficult | Robust and straightforward |
| Security | Not supported | SROS2 (Secure ROS 2) |
| Real-time Support | Limited | Better with DDS QoS |
| Programming Languages | Python, C++ (primary) | Python, C++, Java, etc. (standardized) |
| Threading Model | Single-threaded spin by default | Multi-threaded executor options |
| Message Passing | Asynchronous | Both synchronous and asynchronous |
| Installation | Custom build system (ROSDEB) | Standard package managers |
| Quality of Service | No QoS controls | Rich QoS policies |
| Communication Protocols | TCPROS, UDPROS | DDS protocols (vendor-specific) |

## 2.7 Pro Tips: Working with ROS 2 Communication Patterns

- **Use Topics for streaming data**: Sensor data, robot state, etc.
- **Use Services for simple requests**: Getting robot status, triggering a calibration, etc.
- **Use Actions for long-running tasks**: Navigation, manipulation, trajectory execution
- **Design your messaging architecture early**: Plan your topics, services, and actions before implementation
- **Monitor network traffic**: Use tools like `ros2 topic hz` to monitor message rates
- **Consider the QoS settings**: Different applications have different requirements for reliability, durability, and history
- **Use composition when appropriate**: In some cases, combining related functionality into a single node may be more efficient than using multiple communicating nodes
- **Handle errors gracefully**: Network partitions, node crashes, and other failures should be handled gracefully in your robot applications

## 2.8 Summary

This chapter has covered the core communication patterns in ROS 2: nodes, topics, services, and actions. We've examined how ROS 2's DDS-based architecture addresses the limitations of ROS 1, particularly in multi-robot scenarios and with improved security. The introduction of QoS profiles gives developers fine-grained control over communication behavior.

Understanding these fundamental concepts is crucial for the next chapters, where we'll explore how to bridge AI agents with robots using rclpy and how to describe robots using URDF and Xacro. The communication patterns learned here will be used extensively in creating the "athena" humanoid package in Chapter 5.

## Exercises

1. Create a custom message type and implement a publisher/subscriber for it.
2. Design and implement a service that calculates the distance between two 3D points.
3. Implement an action server that simulates a robot arm movement with progress feedback.
4. Compare the message rate of ROS 1 vs ROS 2 under identical network conditions.
5. Configure QoS settings for a publisher/subscriber pair to guarantee delivery of critical messages.

### Solutions to Exercises

[Detailed solutions would be provided in the exercises appendix]


