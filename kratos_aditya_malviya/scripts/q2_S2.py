#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class S2(Node):
    """
    A ROS2 node that subscribes to the topic /s1.
    Based on the message received ('red' or 'green'),
    it publishes the opposite color to topic /s2.
    """

    def __init__(self):
        super().__init__("q2_S2")
        
        # Publisher for /s2 topic
        self.pub_ = self.create_publisher(String, '/s2', 10)

        # Subscriber to /s1 topic
        self.sub_ = self.create_subscription(String, '/s1', self.check_color, 10)

        # Log startup
        self.get_logger().info('S2 Node started')


    def check_color(self,msg : String):
        """
        Callback for subscription to /s1.
        Publishes the opposite color to /s2.
        """
        if msg.data == 'red':
            msg = String()
            msg.data = 'green'
            self.pub_.publish(msg)
            self.get_logger().info('Published: green')

        else:
            msg = String()
            msg.data = 'red'
            self.pub_.publish(msg)
            self.get_logger().info('Published: red')



def main(args=None):
    rclpy.init(args=args)
    node = S2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()