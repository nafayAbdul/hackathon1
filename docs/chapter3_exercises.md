# Chapter 3 Exercises: rclpy Ã¢â‚¬â€œ Bridging Python AI Agents to Robots

## Exercise 1: AI Node with Camera Processing

**Problem**: Create an AI node that processes camera images using a computer vision model and makes navigation decisions. You don't need to implement the actual computer vision model, but design the ROS 2 node structure and message flow.

**Solution**: 
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import cv2  # For visualization purposes only
import numpy as np

class CameraAINode(Node):
    def __init__(self):
        super().__init__('camera_ai_node')
        
        # Subscribe to camera images
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publisher for robot movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for AI decisions (for logging/visualization)
        self.decision_pub = self.create_publisher(String, '/ai_decision', 10)
        
        self.get_logger().info('Camera AI Node initialized')

    def image_callback(self, msg):
        """Process incoming camera images."""
        # Convert ROS Image message to OpenCV format (simulated)
        # image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # Simulate processing with a placeholder function
        ai_decision = self.process_image_for_navigation(msg.width, msg.height)  # Using dimensions as mock
        
        # Create and publish movement command based on AI decision
        twist_cmd = self.decision_to_twist(ai_decision)
        self.cmd_vel_pub.publish(twist_cmd)
        
        # Log the AI decision
        decision_msg = String()
        decision_msg.data = f'AI decided to {ai_decision}'
        self.decision_pub.publish(decision_msg)
        
        self.get_logger().info(f'AI decision: {ai_decision}')

    def process_image_for_navigation(self, width, height):
        """Placeholder for actual AI processing of image."""
        # In real implementation, this would run an AI model
        # For simulation, return a random decision
        decisions = ['move_forward', 'turn_left', 'turn_right', 'stop']
        import random
        return random.choice(decisions)

    def decision_to_twist(self, decision):
        """Convert AI decision to Twist command."""
        twist = Twist()
        
        if decision == 'move_forward':
            twist.linear.x = 0.2  # Move forward at 0.2 m/s
            twist.angular.z = 0.0
        elif decision == 'turn_left':
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Turn left at 0.5 rad/s
        elif decision == 'turn_right':
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # Turn right at 0.5 rad/s
        elif decision == 'stop':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        return twist

def main(args=None):
    rclpy.init(args=args)
    node = CameraAINode()
    
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

## Exercise 2: Safety Layer for AI Commands

**Problem**: Implement a safety layer that validates AI-generated joint trajectories before execution.

**Solution**:
```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool, String
import numpy as np

class SafetyLayerNode(Node):
    def __init__(self):
        super().__init__('safety_layer_node')
        
        # Subscribe to AI-generated trajectories
        self.ai_traj_sub = self.create_subscription(
            JointTrajectory,
            '/ai_generated_trajectory',
            self.validate_and_forward_trajectory,
            10
        )
        
        # Publisher for validated trajectories
        self.validated_traj_pub = self.create_publisher(
            JointTrajectory,
            '/validated_trajectory',
            10
        )
        
        # Publisher for safety status
        self.safety_status_pub = self.create_publisher(Bool, '/safety_valid', 10)
        
        # Publisher for safety alerts
        self.alert_pub = self.create_publisher(String, '/safety_alerts', 10)
        
        # Define safety constraints
        self.max_joint_velocity = 2.0  # rad/s
        self.max_joint_acceleration = 5.0  # rad/s^2
        self.max_effort = 100.0  # N*m
        self.joint_limits = {  # Example limits for each joint
            'hip_joint': (-1.57, 1.57),
            'knee_joint': (0, 2.0),
            'ankle_joint': (-0.8, 0.8)
        }
        
        self.get_logger().info('Safety Layer Node initialized')

    def validate_and_forward_trajectory(self, msg):
        """Validate the AI-generated trajectory and forward only if safe."""
        is_safe, reason = self.validate_trajectory(msg)
        
        if is_safe:
            # Publish validated trajectory
            self.validated_traj_pub.publish(msg)
            # Publish safety status (valid)
            status_msg = Bool()
            status_msg.data = True
            self.safety_status_pub.publish(status_msg)
            self.get_logger().info('Trajectory validated as safe, forwarding to robot')
        else:
            # Publish alert about unsafe trajectory
            alert_msg = String()
            alert_msg.data = f'Safety violation: {reason}'
            self.alert_pub.publish(alert_msg)
            
            # Publish safety status (invalid)
            status_msg = Bool()
            status_msg.data = False
            self.safety_status_pub.publish(status_msg)
            
            self.get_logger().error(f'Unsafe trajectory blocked: {reason}')

    def validate_trajectory(self, traj):
        """Validate trajectory against safety constraints."""
        for point_idx, point in enumerate(traj.points):
            # Check position limits
            for joint_idx, pos in enumerate(point.positions):
                if joint_idx  max_limit:
                            return False, f'Joint {joint_name} position {pos} exceeds limits [{min_limit}, {max_limit}] at point {point_idx}'
            
            # Check velocity limits
            if point.velocities:
                for vel in point.velocities:
                    if abs(vel) > self.max_joint_velocity:
                        return False, f'Velocity {vel} exceeds maximum {self.max_joint_velocity} at point {point_idx}'
            
            # Check acceleration limits
            if point.accelerations:
                for acc in point.accelerations:
                    if abs(acc) > self.max_joint_acceleration:
                        return False, f'Acceleration {acc} exceeds maximum {self.max_joint_acceleration} at point {point_idx}'
            
            # Check effort limits
            if point.effort:
                for effort in point.effort:
                    if abs(effort) > self.max_effort:
                        return False, f'Effort {effort} exceeds maximum {self.max_effort} at point {point_idx}'
        
        return True, "Valid trajectory"

def main(args=None):
    rclpy.init(args=args)
    node = SafetyLayerNode()
    
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

## Exercise 3: Caching AI Responses

**Problem**: Design a system that caches responses from slow AI models to improve responsiveness.

**Solution**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import time
from collections import OrderedDict

class CachedAINode(Node):
    def __init__(self):
        super().__init__('cached_ai_node')
        
        # Subscriber for input queries
        self.query_sub = self.create_subscription(
            String,
            '/ai_query',
            self.handle_query,
            10
        )
        
        # Publisher for AI responses
        self.response_pub = self.create_publisher(String, '/ai_response', 10)
        
        # Initialize cache (LRU cache for efficiency)
        self.cache_size = 10
        self.query_cache = OrderedDict()
        
        self.get_logger().info('Cached AI Node initialized')

    def handle_query(self, msg):
        """Handle incoming query, returning cached response if available."""
        query = msg.data
        
        # Check if query is in cache
        if query in self.query_cache:
            # Get cached response
            response = self.query_cache[query]
            
            # Move to end (most recently used)
            self.query_cache.move_to_end(query)
            
            self.get_logger().info(f'Returning cached response for: {query}')
        else:
            # Process query (simulating slow AI model)
            response = self.process_query_slowly(query)
            
            # Add to cache
            self.add_to_cache(query, response)
            
            self.get_logger().info(f'Processed new query: {query}')
        
        # Publish response
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

    def process_query_slowly(self, query):
        """Simulate a slow AI model processing."""
        self.get_logger().info(f'Processing slow query: {query}')
        
        # Simulate processing time
        time.sleep(0.5)
        
        # Return a response
        return f"Response to: {query} (processed at {time.time():.2f})"

    def add_to_cache(self, query, response):
        """Add result to cache, removing oldest if necessary."""
        # Add new item
        self.query_cache[query] = response
        
        # If cache is too large, remove oldest item
        if len(self.query_cache) > self.cache_size:
            self.query_cache.popitem(last=False)

def main(args=None):
    rclpy.init(args=args)
    node = CachedAINode()
    
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

## Exercise 4: Resource Monitoring Node

**Problem**: Create a node that monitors CPU and memory usage of AI processes and throttles when resources are low.

**Solution**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
import psutil

class ResourceMonitorNode(Node):
    def __init__(self):
        super().__init__('resource_monitor_node')
        
        # Publishers for resource readings
        self.cpu_usage_pub = self.create_publisher(Float32, '/cpu_usage_percent', 10)
        self.memory_usage_pub = self.create_publisher(Float32, '/memory_usage_percent', 10)
        self.throttle_signal_pub = self.create_publisher(Bool, '/ai_throttle', 10)
        
        # Timer for periodic monitoring
        self.timer = self.create_timer(1.0, self.monitor_resources)
        
        # Resource thresholds
        self.cpu_threshold = 80.0  # percent
        self.memory_threshold = 85.0  # percent
        
        self.get_logger().info('Resource Monitor Node initialized')

    def monitor_resources(self):
        """Monitor system resources and publish readings."""
        # Get CPU usage
        cpu_percent = psutil.cpu_percent(interval=None)
        
        # Get memory usage
        memory_info = psutil.virtual_memory()
        memory_percent = memory_info.percent
        
        # Publish readings
        cpu_msg = Float32()
        cpu_msg.data = cpu_percent
        self.cpu_usage_pub.publish(cpu_msg)
        
        memory_msg = Float32()
        memory_msg.data = memory_percent
        self.memory_usage_pub.publish(memory_msg)
        
        # Check if resources are low and publish throttle signal
        should_throttle = (
            cpu_percent > self.cpu_threshold or
            memory_percent > self.memory_threshold
        )
        
        throttle_msg = Bool()
        throttle_msg.data = should_throttle
        self.throttle_signal_pub.publish(throttle_msg)
        
        # Log status
        status = "RESOURCES_OK"
        if should_throttle:
            status = f"THROTTLING_CPU:{cpu_percent:.1f}%_MEM:{memory_percent:.1f}%"
        elif cpu_percent > 70 or memory_percent > 75:
            status = f"WARNING_CPU:{cpu_percent:.1f}%_MEM:{memory_percent:.1f}%"
            
        self.get_logger().info(f'Resources: CPU={cpu_percent:.1f}%, MEM={memory_percent:.1f}% [{status}]')

def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitorNode()
    
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

## Exercise 5: Fallback Mechanism

**Problem**: Implement a fallback mechanism that activates manual control when AI services fail.

**Solution**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import time

class FallbackMechanismNode(Node):
    def __init__(self):
        super().__init__('fallback_mechanism_node')
        
        # Subscriber for AI service status
        self.ai_status_sub = self.create_subscription(
            Bool,
            '/ai_service_active',
            self.ai_status_callback,
            10
        )
        
        # Subscriber for manual control override
        self.manual_override_sub = self.create_subscription(
            Bool,
            '/manual_override',
            self.manual_override_callback,
            10
        )
        
        # Publisher for active control mode
        self.mode_publisher = self.create_publisher(String, '/control_mode', 10)
        
        # Publisher for robot commands (will switch source based on mode)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher for AI commands (simulated)
        self.ai_cmd_publisher = self.create_publisher(Twist, '/ai_cmd_vel', 10)
        
        # Publisher for manual commands (simulated input)
        self.manual_cmd_publisher = self.create_publisher(Twist, '/manual_cmd_vel', 10)
        
        # Initialize state
        self.ai_active = True
        self.manual_override = False
        self.last_ai_status_time = time.time()
        self.ai_timeout_threshold = 3.0  # seconds without status = AI failure
        
        # Timer for checking system health
        self.health_timer = self.create_timer(0.5, self.check_system_health)
        
        self.update_control_mode()
        self.get_logger().info('Fallback Mechanism Node initialized')

    def ai_status_callback(self, msg):
        """Callback for AI service status."""
        self.ai_active = msg.data
        self.last_ai_status_time = time.time()
        self.get_logger().info(f'AI status updated: {self.ai_active}')
        self.update_control_mode()

    def manual_override_callback(self, msg):
        """Callback for manual override."""
        self.manual_override = msg.data
        self.get_logger().info(f'Manual override: {self.manual_override}')
        self.update_control_mode()

    def check_system_health(self):
        """Check if AI service is responsive."""
        current_time = time.time()
        time_since_last_status = current_time - self.last_ai_status_time
        
        if time_since_last_status > self.ai_timeout_threshold and self.ai_active:
            # AI service seems unresponsive
            self.ai_active = False
            self.get_logger().warn('AI service timed out, switching to fallback mode')
            self.update_control_mode()

    def update_control_mode(self):
        """Update control mode based on current state."""
        if self.manual_override:
            mode = 'MANUAL'
        elif self.ai_active:
            mode = 'AI_CONTROLLED'
        else:
            mode = 'SAFETY_FALLBACK'
        
        # Publish current mode
        mode_msg = String()
        mode_msg.data = mode
        self.mode_publisher.publish(mode_msg)
        
        self.get_logger().info(f'Control mode: {mode}')
        
        # Depending on mode, different command sources are used
        if mode == 'AI_CONTROLLED':
            # Use AI commands (in real system, this would be a relay)
            pass
        elif mode == 'MANUAL':
            # Use manual commands
            pass
        else:  # SAFETY_FALLBACK
            # In safety fallback, send zero velocity commands
            zero_cmd = Twist()
            self.cmd_publisher.publish(zero_cmd)
            self.get_logger().warn('Safety fallback active - robot stopped')

def main(args=None):
    rclpy.init(args=args)
    node = FallbackMechanismNode()
    
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


