#!/usr/bin/env python3

"""
Chapter 2: Basic Action Client-Server Example
This example demonstrates the basic action communication pattern in ROS 2.
"""

import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.action import Fibonacci
import time


class FibonacciActionServer(Node):
    """
    A simple action server node that implements the Fibonacci action.
    """

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        """
        Called to accept or reject a goal request.
        """
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Called when a goal is cancelled.
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Executes the goal and provides feedback along the way.
        """
        self.get_logger().info('Executing goal...')

        # Create feedback and result messages
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        result_msg = Fibonacci.Result()

        # Send feedback periodically while calculating
        for i in range(1, goal_handle.request.order):
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result_msg.sequence = feedback_msg.sequence
                return result_msg

            # Check if the node has been shutdown
            if not self.handle:
                result_msg.sequence = feedback_msg.sequence
                return result_msg

            # Calculate next Fibonacci number
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Simulate some work by sleeping
            time.sleep(0.5)

        # Complete the goal
        goal_handle.succeed()
        result_msg.sequence = feedback_msg.sequence

        self.get_logger().info(f'Result: {result_msg.sequence}')
        return result_msg


class FibonacciActionClient(Node):
    """
    A simple action client node that calls the Fibonacci action.
    """

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order):
        """
        Send a goal to the action server.
        """
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Handle the response when the goal is accepted or rejected.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Handle feedback messages from the action server.
        """
        self.get_logger().info(f'Received feedback: {feedback_msg.sequence}')

    def get_result_callback(self, future):
        """
        Handle the final result from the action server.
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

        # Shutdown after receiving the result
        rclpy.shutdown()


def main():
    """
    Main function that demonstrates both action server and client.
    """
    rclpy.init()

    # Create nodes
    action_server = FibonacciActionServer()
    action_client = FibonacciActionClient()

    # Create executor for both nodes
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(action_server)
    executor.add_node(action_client)

    # Send goal from client after nodes start
    def send_goal():
        time.sleep(1)  # Give server time to start
        action_client.send_goal(10)

    # Schedule goal to be sent
    import threading
    timer = threading.Timer(1.0, send_goal)
    timer.start()

    # Run both nodes
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_server.destroy_node()
        action_client.destroy_node()
        rclpy.shutdown()
        timer.cancel()


if __name__ == '__main__':
    main()