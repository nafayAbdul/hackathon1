#!/usr/bin/env python3

"""
Chapter 2: Basic Service Client-Server Example
This example demonstrates the basic service communication pattern in ROS 2.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class MinimalService(Node):
    """
    A simple service node that provides addition service.
    """

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        """
        Callback function for the service that adds two integers.
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning {request.a} + {request.b} = {response.sum}')
        return response


class MinimalClientAsync(Node):
    """
    A simple client node that calls the addition service.
    """

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to the service.
        """
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future


def main():
    """
    Main function that demonstrates both service server and client.
    This function runs the server and client in separate threads.
    """
    rclpy.init()

    # Create the service node
    service_node = MinimalService()
    
    # Create the client node in a separate thread
    import threading
    import time

    def client_thread():
        """
        Client thread that calls the service after a delay.
        """
        time.sleep(2)  # Give the service time to start
        client_node = MinimalClientAsync()
        
        # Send a request
        future = client_node.send_request(1, 2)
        
        # Wait for response
        rclpy.spin_until_future_completed(client_node, future)
        response = future.result()
        if response is not None:
            client_node.get_logger().info(f'Result of add_two_ints: {response.sum}')
        else:
            client_node.get_logger().info('No response received')
        
        client_node.destroy_node()

    # Start the client in a separate thread
    client_thread_obj = threading.Thread(target=client_thread)
    client_thread_obj.start()

    # Spin the service node
    try:
        rclpy.spin(service_node)
    except KeyboardInterrupt:
        pass
    finally:
        # Wait for client thread to complete
        client_thread_obj.join()
        service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()