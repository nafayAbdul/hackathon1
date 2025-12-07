#!/usr/bin/env python3

"""
Chapter 3: Error Handling Examples
This example demonstrates how to handle various errors in AI-robot communication systems.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
import time
import threading
import requests
from athena_interfaces.msg import AICommand  # Using custom AI command message


class RobustAINode(Node):
    """
    A node that demonstrates robust error handling for AI-robot communication.
    """

    def __init__(self):
        super().__init__('robust_ai_node')
        
        # Publisher for error status
        self.error_status_publisher = self.create_publisher(Bool, 'ai_error_status', 10)
        
        # Publisher for error messages
        self.error_log_publisher = self.create_publisher(String, 'error_log', 10)
        
        # Timer for periodic AI tasks (with timeout handling)
        self.timer = self.create_timer(2.0, self.periodic_ai_task)
        
        # Track error state
        self.error_occurred = False
        
        self.get_logger().info('Robust AI Node initialized')

    def periodic_ai_task(self):
        """
        Perform a periodic AI task with proper error handling.
        """
        try:
            # Simulate a call that might timeout
            response = self.call_external_ai_service(timeout=1.5)
            
            if response and response.get('success'):
                self.get_logger().info(f'AI task completed successfully: {response.get("data", "")}')
                self.publish_error_status(False)  # No error
            else:
                error_msg = response.get('error', 'Unknown error') if response else 'No response from AI service'
                self.get_logger().warn(f'AI service returned error: {error_msg}')
                self.publish_error_status(True)  # Error occurred
                
        except TimeoutError as e:
            self.get_logger().error(f'AI service timed out: {str(e)}')
            self.handle_timeout_error(str(e))
        except requests.exceptions.ConnectionError as e:
            self.get_logger().error(f'Connection error to AI service: {str(e)}')
            self.handle_connection_error(str(e))
        except Exception as e:
            self.get_logger().error(f'Unexpected error in AI task: {str(e)}')
            self.handle_general_error(str(e))

    def call_external_ai_service(self, timeout=1.0):
        """
        Simulate calling an external AI service that might timeout.
        """
        # In a real implementation, this would make an actual API call:
        # response = requests.post(
        #     'http://localhost:8000/predict',
        #     json={'data': 'input_data'},
        #     timeout=timeout
        # )
        
        # For this template, we'll simulate the behavior
        import random
        
        # Simulate occasional timeouts
        if random.uniform(0, 1) < 0.1:  # 10% chance of timeout
            raise TimeoutError("External AI service timed out")
        
        # Simulate occasional connection errors
        if random.uniform(0, 1) < 0.05:  # 5% chance of connection error
            raise requests.exceptions.ConnectionError("Failed to connect to AI service")
        
        # Return a successful response
        return {
            'success': True,
            'data': f'AI response at {time.time()}',
            'error': None
        }

    def handle_timeout_error(self, error_msg):
        """
        Handle timeout errors specifically.
        """
        self.get_logger().info('Activating timeout recovery protocols')
        
        # Log the error
        error_log_msg = String()
        error_log_msg.data = f'TIMEOUT_ERROR: {error_msg}'
        self.error_log_publisher.publish(error_log_msg)
        
        # Set error status and try to recover
        self.publish_error_status(True)
        
        # Attempt recovery after timeout
        self.attempt_recovery_after_timeout()

    def handle_connection_error(self, error_msg):
        """
        Handle connection errors specifically.
        """
        self.get_logger().info('Activating connection error recovery protocols')
        
        # Log the error
        error_log_msg = String()
        error_log_msg.data = f'CONNECTION_ERROR: {error_msg}'
        self.error_log_publisher.publish(error_log_msg)
        
        # Set error status
        self.publish_error_status(True)

    def handle_general_error(self, error_msg):
        """
        Handle general errors.
        """
        self.get_logger().info('Activating general error recovery protocols')
        
        # Log the error
        error_log_msg = String()
        error_log_msg.data = f'GENERAL_ERROR: {error_msg}'
        self.error_log_publisher.publish(error_log_msg)
        
        # Set error status
        self.publish_error_status(True)

    def attempt_recovery_after_timeout(self):
        """
        Attempt recovery after a timeout error.
        """
        # In a real system, you might try to restart services, reconnect, etc.
        self.get_logger().info('Attempting recovery after timeout...')
        
        # Wait a bit before trying again
        time.sleep(2.0)
        
        # Reset error status after recovery attempt
        self.publish_error_status(False)
        self.get_logger().info('Recovery attempt completed')

    def publish_error_status(self, has_error):
        """
        Publish the current error status.
        """
        error_status_msg = Bool()
        error_status_msg.data = has_error
        self.error_status_publisher.publish(error_status_msg)


class SensorSafetyNode(Node):
    """
    A node that demonstrates safety measures for sensor failures.
    """

    def __init__(self):
        super().__init__('sensor_safety_node')
        
        # Subscribe to joint states
        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.joint_state_subscription  # Prevent unused variable warning
        
        # Publisher for safety status
        self.safety_status_publisher = self.create_publisher(Bool, 'safety_status', 10)
        
        # Track sensor data freshness
        self.last_sensor_update_time = None
        self.max_sensor_age = 1.0  # seconds
        
        # Timer to check sensor health
        self.health_check_timer = self.create_timer(0.5, self.check_sensor_health)

        self.get_logger().info('Sensor Safety Node initialized')

    def joint_state_callback(self, msg):
        """
        Callback for processing joint state data.
        """
        self.last_sensor_update_time = self.get_clock().now()
        self.get_logger().debug(f'Received joint states for {len(msg.name)} joints')

    def check_sensor_health(self):
        """
        Check if sensor data is fresh and publish safety status.
        """
        if self.last_sensor_update_time is None:
            # Sensors never updated - error state
            self.get_logger().error('Sensors never updated - no data received')
            self.publish_safety_status(False)
            return
            
        current_time = self.get_clock().now()
        sensor_age_ns = current_time.nanoseconds - self.last_sensor_update_time.nanoseconds
        sensor_age_s = sensor_age_ns / 1e9
        
        if sensor_age_s > self.max_sensor_age:
            # Sensor data too old - error state
            self.get_logger().error(f'Sensor data is {sensor_age_s:.2f}s old - exceeding threshold of {self.max_sensor_age}s')
            self.publish_safety_status(False)
        else:
            # Sensor data is fresh - safe state
            self.publish_safety_status(True)

    def publish_safety_status(self, is_safe):
        """
        Publish the current safety status.
        """
        safety_msg = Bool()
        safety_msg.data = is_safe
        self.safety_status_publisher.publish(safety_msg)


def main(args=None):
    """
    Main function that initializes the nodes and spins them with error handling.
    """
    rclpy.init(args=args)

    # Create the robust AI node
    robust_ai_node = RobustAINode()
    
    # Create the sensor safety node
    sensor_safety_node = SensorSafetyNode()
    
    # Create an executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robust_ai_node)
    executor.add_node(sensor_safety_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'An unexpected error occurred in the executor: {e}')
        # Log error, attempt graceful shutdown
        robust_ai_node.get_logger().error(f'Executor error: {str(e)}')
    finally:
        # Always clean up
        robust_ai_node.destroy_node()
        sensor_safety_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()