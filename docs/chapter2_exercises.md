# Chapter 2 Exercises: ROS 2 Humble/Iron Deep Dive

## Exercise 1: Custom Message Type

**Problem**: Create a custom message type called "Temperature" with fields for temperature (float64), unit (string), and timestamp (builtin_interfaces/Time). Implement a publisher and subscriber for this message type.

**Solution**:
1. Create a directory for custom messages: `msg/Temperature.msg` with content:
```
float64 temperature
string unit
builtin_interfaces/Time timestamp
```

2. Create a publisher in Python:
```python
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from your_package_name.msg import Temperature  # Assuming your custom message is in the package

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher = self.create_publisher(Temperature, 'temperature_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Temperature()
        msg.temperature = 25.0
        msg.unit = "Celsius"
        
        # Get current time
        now = self.get_clock().now()
        msg.timestamp.sec = now.nanoseconds // 1000000000
        msg.timestamp.nanosec = now.nanoseconds % 1000000000
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.temperature} {msg.unit}')

def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    
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

3. Create a subscriber in Python:
```python
import rclpy
from rclpy.node import Node
from your_package_name.msg import Temperature

class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Temperature,
            'temperature_topic',
            self.listener_callback,
            10)  # QoS history depth
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Temperature: {msg.temperature} {msg.unit}, Timestamp: {msg.timestamp.sec}.{msg.timestamp.nanosec}')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSubscriber()
    
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

## Exercise 2: Distance Calculation Service

**Problem**: Design a service that calculates the distance between two 3D points and implement both the server and client.

**Solution**:
1. Define the service in `srv/CalculateDistance.srv`:
```
# Input: two 3D points
geometry_msgs/Point point1
geometry_msgs/Point point2
---
# Output: the distance between them
float64 distance
```

2. Service server implementation:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from your_package_name.srv import CalculateDistance  # Using your custom service
import math

class DistanceCalculator(Node):
    def __init__(self):
        super().__init__('distance_calculator')
        self.srv = self.create_service(CalculateDistance, 'calculate_distance', self.calculate_distance_callback)

    def calculate_distance_callback(self, request, response):
        # Calculate Euclidean distance
        dx = request.point1.x - request.point2.x
        dy = request.point1.y - request.point2.y
        dz = request.point1.z - request.point2.z
        
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        response.distance = distance
        
        self.get_logger().info(
            f'Distance between ({request.point1.x:.2f}, {request.point1.y:.2f}, {request.point1.z:.2f}) and '
            f'({request.point2.x:.2f}, {request.point2.y:.2f}, {request.point2.z:.2f}): {distance:.2f}'
        )
        
        return response

def main(args=None):
    rclpy.init(args=args)
    calculator = DistanceCalculator()
    
    try:
        rclpy.spin(calculator)
    except KeyboardInterrupt:
        pass
    finally:
        calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. Service client implementation:
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from your_package_name.srv import CalculateDistance
import sys

class DistanceClient(Node):
    def __init__(self):
        super().__init__('distance_client')
        self.cli = self.create_client(CalculateDistance, 'calculate_distance')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = CalculateDistance.Request()

    def send_request(self, x1, y1, z1, x2, y2, z2):
        self.req.point1 = Point(x=x1, y=y1, z=z1)
        self.req.point2 = Point(x=x2, y=y2, z=z2)
        
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = DistanceClient()

    # Example: Calculate distance between (0,0,0) and (3,4,0) - should be 5
    future = client.send_request(0.0, 0.0, 0.0, 3.0, 4.0, 0.0)

    try:
        rclpy.spin_until_future_completed(client, future)
        response = future.result()
        if response is not None:
            client.get_logger().info(f'Distance calculated: {response.distance}')
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

## Exercise 3: Robot Arm Action Server

**Problem**: Implement an action server that simulates a robot arm movement with progress feedback.

**Solution**:
1. Define action in `action/Motion.action`:
```
# Goal
float64[] target_positions
---
# Result
bool success
string message
---
# Feedback
float64[] current_positions
string status
```

2. Action server implementation:
```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from your_package_name.action import Motion  # Using your custom action
import time

class ArmMotionActionServer(Node):
    def __init__(self):
        super().__init__('arm_motion_action_server')
        self._action_server = ActionServer(
            self,
            Motion,
            'arm_motion',
            execute_callback=self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing arm motion goal...')
        
        # Current position starts at [0, 0, 0, 0, 0]
        current_pos = [0.0, 0.0, 0.0, 0.0, 0.0]
        target_pos = list(goal_handle.request.target_positions)
        
        feedback_msg = Motion.Feedback()
        feedback_msg.current_positions = current_pos[:]
        feedback_msg.status = "Moving..."
        
        # Simulate movement over 10 steps
        steps = 10
        for i in range(steps):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Motion.Result(success=False, message='Goal canceled')
            
            # Check if still active
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return Motion.Result(success=False, message='Goal aborted')
            
            # Move towards target
            for j in range(len(current_pos)):
                current_pos[j] += (target_pos[j] - current_pos[j]) / (steps - i)
            
            # Update feedback
            feedback_msg.current_positions = current_pos[:]
            feedback_msg.status = f"Moving... Step {i+1}/{steps}"
            goal_handle.publish_feedback(feedback_msg)
            
            self.get_logger().info(f'Publishing feedback: {feedback_msg.current_positions}')
            
            # Sleep to simulate movement time
            time.sleep(0.1)
        
        # Goal succeeded
        goal_handle.succeed()
        result = Motion.Result()
        result.success = True
        result.message = "Arm movement completed successfully"
        self.get_logger().info('Arm movement completed')
        return result

def main(args=None):
    rclpy.init(args=args)
    arm_action_server = ArmMotionActionServer()
    
    try:
        rclpy.spin(arm_action_server)
    except KeyboardInterrupt:
        pass
    finally:
        arm_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Exercise 4: QoS Configuration

**Problem**: Configure QoS settings for a publisher/subscriber pair to guarantee delivery of critical messages.

**Solution**:
QoS settings for reliable, durable communication:

Publisher code:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class ReliablePublisher(Node):
    def __init__(self):
        super().__init__('reliable_publisher')
        
        # Configure QoS profile
        qos_profile = QoSProfile(
            depth=10,  # History depth
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Store for late joiners
            reliability=QoSReliabilityPolicy.RELIABLE  # Guarantee delivery
        )
        
        self.publisher = self.create_publisher(String, 'critical_topic', qos_profile)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = f'Critical message: {self.get_clock().now()}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing critically: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ReliablePublisher()
    
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
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class ReliableSubscriber(Node):
    def __init__(self):
        super().__init__('reliable_subscriber')
        
        # Configure QoS profile to match publisher
        qos_profile = QoSProfile(
            depth=10,  # History depth
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Store for late joiners
            reliability=QoSReliabilityPolicy.RELIABLE  # Guarantee delivery
        )
        
        self.subscription = self.create_subscription(
            String,
            'critical_topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Critically heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = ReliableSubscriber()
    
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

## Exercise 5: Lifecycle Nodes

**Problem**: Explain when you would use a lifecycle node instead of a regular node.

**Solution**:
Lifecycle nodes are useful in the following scenarios:

1. **Complex systems with coordinated startup**: In robotic systems with multiple interconnected components, you might want to ensure that sensors are properly initialized and calibrated before the perception nodes start processing data.

2. **Resource management**: For nodes that manage significant resources (large data buffers, GPU memory, hardware interfaces), the lifecycle provides a clean way to allocate and de-allocate these resources appropriately.

3. **Hardware integration**: When integrating with real hardware, you might need to go through specific configuration steps before the hardware is ready to be used.

4. **Multi-robot coordination**: In multi-robot systems, lifecycle nodes can help coordinate the activation and deactivation of different robots or components.

5. **Safety-critical applications**: The lifecycle allows for specific safety checks at each stage of activation, ensuring that a node is properly configured and safe to activate before becoming active.

Regular nodes are simpler and appropriate for lightweight components that don't require careful resource management or coordinated startup.


