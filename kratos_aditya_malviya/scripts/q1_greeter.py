#!/usr/bin/env python3

# Import necessary ROS 2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Greeter(Node):
    """
    A ROS2 node that publishes "Hello World!" to the /new topic
    at a fixed rate of 15 messages per second.
    """

    def __init__(self):
        super().__init__("q1_greeter")  # Initialize the node with name 'q1_greeter'

        # Create publisher to publish String messages on '/new' topic
        self.publisher_ = self.create_publisher(String, '/new', 10)

        # Define the timer interval for 15 Hz (1/15 seconds)
        publish_interval = 1.0 / 15.0

        # Create a timer that triggers the callback at the specified interval
        self.timer = self.create_timer(publish_interval, self.pub_greetings)

        self.get_logger().info("Greeter node started. Publishing 'Hello World!' at 15 Hz.")

    def pub_greetings(self):
        """
        Timer callback function to publish "Hello World!" message.
        Called 15 times per second.
        """
        msg = String()
        msg.data = "Hello World !"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    """
    Entry point for the ROS2 node. Initializes and spins the node.
    """
    rclpy.init(args=args)
    node = Greeter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
