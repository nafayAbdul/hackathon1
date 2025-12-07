#!/usr/bin/env python3

"""
Chapter 3: Latency Measurement Tools
This example demonstrates how to measure latency in AI-robot communication systems.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float64
from builtin_interfaces.msg import Time
import time
from collections import deque
import statistics


class LatencyMeasurementNode(Node):
    """
    A node that measures and reports latency in the system.
    """

    def __init__(self):
        super().__init__('latency_measurement_node')
        
        # Publisher for timestamped test messages
        self.test_publisher = self.create_publisher(Header, 'latency_test', 10)
        
        # Subscription to echoed test messages
        self.test_subscription = self.create_subscription(
            Header,
            'latency_test_echo',  # In real system, an echo node would republish this
            self.latency_response_callback,
            10
        )
        self.test_subscription  # Prevent unused variable warning
        
        # Publisher for latency results
        self.latency_publisher = self.create_publisher(Float64, 'latency_results', 10)
        
        # Timer to send test messages periodically
        self.timer = self.create_timer(2.0, self.send_latency_test_message)
        
        # Storage for tracking message timestamps
        self.sent_times = {}
        self.message_counter = 0
        
        # Statistics for latency measurements
        self.latency_samples = deque(maxlen=100)  # Keep last 100 samples
        
        # Stats reporting timer
        self.stats_timer = self.create_timer(10.0, self.report_statistics)
        
        self.get_logger().info('Latency Measurement Node initialized')

    def send_latency_test_message(self):
        """
        Send a test message with a timestamp to measure round-trip time.
        """
        # Create a header with timestamp
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = f"latency_test_{self.message_counter}"
        
        # Record the time we sent the message (using high-resolution timer)
        send_time = time.perf_counter()
        self.sent_times[header.frame_id] = send_time
        self.message_counter += 1
        
        self.test_publisher.publish(header)
        self.get_logger().info(f'Sent latency test message: {header.frame_id}')

    def latency_response_callback(self, msg):
        """
        Callback for receiving echoed test messages and calculating latency.
        """
        # Get the precise time the message was received
        receive_time = time.perf_counter()
        
        # Look up the time the message was sent
        sent_time = self.sent_times.pop(msg.frame_id, None)
        
        if sent_time is not None:
            # Calculate round-trip time in milliseconds
            rtt_ms = (receive_time - sent_time) * 1000
            
            # Store the latency sample
            self.latency_samples.append(rtt_ms)
            
            # Publish the latency result
            latency_msg = Float64()
            latency_msg.data = rtt_ms
            self.latency_publisher.publish(latency_msg)
            
            self.get_logger().info(f'Latency measurement for {msg.frame_id}: {rtt_ms:.2f}ms')
        else:
            self.get_logger().warn(f'Received echo for unknown message: {msg.frame_id}')

    def report_statistics(self):
        """
        Report statistics about the latency measurements.
        """
        if not self.latency_samples:
            self.get_logger().info('No latency samples to report')
            return

        # Calculate statistics
        avg_latency = statistics.mean(self.latency_samples)
        min_latency = min(self.latency_samples)
        max_latency = max(self.latency_samples)
        
        # Calculate 95th percentile (approximation)
        sorted_latencies = sorted(self.latency_samples)
        p95_index = int(0.95 * len(sorted_latencies))
        p95_latency = sorted_latencies[min(p95_index, len(sorted_latencies)-1)] if sorted_latencies else 0.0
        
        # Report statistics
        self.get_logger().info('=== LATENCY STATISTICS ===')
        self.get_logger().info(f'Samples: {len(self.latency_samples)}, Avg: {avg_latency:.2f}ms')
        self.get_logger().info(f'Min: {min_latency:.2f}ms, Max: {max_latency:.2f}ms, 95th%: {p95_latency:.2f}ms')
        
        # Check if latency meets requirements
        if p95_latency > 100.0:
            self.get_logger().error('CRITICAL: 95th percentile latency exceeds 100ms threshold!')
        elif p95_latency > 50.0:
            self.get_logger().warn('WARNING: 95th percentile latency approaching 50ms threshold')
        else:
            self.get_logger().info('OK: Latency within acceptable range')


class AIProcessingLatencyNode(Node):
    """
    A node that measures the latency of AI processing.
    """

    def __init__(self):
        super().__init__('ai_processing_latency_node')
        
        # Subscription to input data
        self.input_subscription = self.create_subscription(
            Header,
            'ai_input_data',
            self.process_input_with_timing,
            10
        )
        self.input_subscription  # Prevent unused variable warning
        
        # Publisher for processed data
        self.output_publisher = self.create_publisher(Header, 'ai_output_data', 10)
        
        # Publisher for latency measurements
        self.latency_publisher = self.create_publisher(Float64, 'ai_processing_latency', 10)
        
        self.get_logger().info('AI Processing Latency Node initialized')

    def process_input_with_timing(self, msg):
        """
        Process input data and measure the processing time.
        """
        # Record start time
        start_time = time.perf_counter()
        
        # Simulate AI processing (in real implementation, this would be an actual model)
        self.simulate_ai_processing()
        
        # Calculate processing time in milliseconds
        processing_time_ms = (time.perf_counter() - start_time) * 1000
        
        # Publish the processing time
        latency_msg = Float64()
        latency_msg.data = processing_time_ms
        self.latency_publisher.publish(latency_msg)
        
        # Create and publish output message
        output_msg = Header()
        output_msg.stamp = self.get_clock().now().to_msg()
        output_msg.frame_id = f"processed_{msg.frame_id}_latency_{processing_time_ms:.2f}ms"
        
        self.output_publisher.publish(output_msg)
        
        self.get_logger().info(f'AI processing took {processing_time_ms:.2f}ms for {msg.frame_id}')
        
        # Warn if processing time is too high
        if processing_time_ms > 100.0:
            self.get_logger().warn(f'AI processing time exceeded 100ms threshold: {processing_time_ms:.2f}ms')

    def simulate_ai_processing(self):
        """
        Simulate AI processing time (in real implementation, this would be actual AI inference).
        """
        # Simulate some processing time (in a real implementation, this would be AI inference)
        time.sleep(0.02)  # Simulate 20ms of processing


def main(args=None):
    """
    Main function that initializes the nodes and spins them.
    """
    rclpy.init(args=args)

    # Create the latency measurement node
    latency_node = LatencyMeasurementNode()
    
    # Create the AI processing latency node
    ai_latency_node = AIProcessingLatencyNode()
    
    # Create an executor to handle both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(latency_node)
    executor.add_node(ai_latency_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        latency_node.destroy_node()
        ai_latency_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()