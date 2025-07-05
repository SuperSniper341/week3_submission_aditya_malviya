#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Greeter(Node):

    def __init__(self): #initializing node
        super().__init__("q1_greeter")
        self.publisher_ = self.create_publisher(String,'/new',10)
        time = 1.0/15.0
        self.timer = self.create_timer(time , self.pub_greetings)

    def pub_greetings(self): # callback function called 15 times per second
        msg = String()
        msg.data = "Hello World !"
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = Greeter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()