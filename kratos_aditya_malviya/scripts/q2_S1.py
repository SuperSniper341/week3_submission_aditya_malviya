#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class S1(Node): 
    """
    A ROS2 node that alternately publishes 'green' and 'red'
    messages to the topic /s1 every 10 seconds.
    """

    def __init__(self):
        super().__init__("q2_S1")

        # Publisher for /s1 topic
        self.pub_ = self.create_publisher(String, '/s1', 10)

        # Log startup
        self.get_logger().info('S1 Node started')

        # Initial color
        self.current_color = 'red'

        # Publish the first color immediately
        self.publish_color()

        # Create a timer to call publish_color every 10 seconds
        self.timer = self.create_timer(10.0, self.publish_color)
        
    def publish_color(self):
        """
        Publishes the current color and toggles it for the next call.
        """
        msg = String()
        msg.data = self.current_color
        self.pub_.publish(msg)
        self.get_logger().info(f'Published: {self.current_color}')

        # Toggle color for the next cycle
        self.current_color = 'green' if self.current_color == 'red' else 'red'


def main(args=None):
    rclpy.init(args=args)
    node = S1()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
