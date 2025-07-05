#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class S1(Node): 

    def __init__(self):
        super().__init__("q2_S1") 
        self.pub_ = self.create_publisher(String,'/s1',10)

        self.get_logger().info('S1 Node started')

        msg = String()
        msg.data = 'red'
        self.pub_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

        self.timer = self.create_timer(10.0, self.publish_color)
        self.current_color = 'green'
        
    def publish_color(self):
        msg = String()
        msg.data = self.current_color
        
        self.pub_.publish(msg)
        self.get_logger().info(f'Published: {self.current_color}')

        if self.current_color == 'red':
            self.current_color = 'green'
        else:
            self.current_color = 'red'

def main(args=None):
    rclpy.init(args=args)
    node = S1() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()