#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class S2(Node):
    def __init__(self):
        super().__init__("q2_S2")
        self.pub_ = self.create_publisher(String,'/s2',10)
        self.sub_ = self.create_subscription(String,'/s1',self.check_color,10)
        self.get_logger().info('S2 Node started')


    def check_color(self,msg : String):
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