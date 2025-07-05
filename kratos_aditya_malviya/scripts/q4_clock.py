#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32


class Clock(Node):
    def __init__(self):
        super().__init__("q4_clock")
        self.sec_ = 0
        self.min_ = 0
        self.hour_ = 0

        self.sec_pub_ = self.create_publisher(Int32 , '/second' ,10)
        self.min_pub_ = self.create_publisher(Int32 , '/minute' ,10)
        self.hour_pub_ = self.create_publisher(Int32 , '/hour' ,10)

        self.sec_sub_ = self.create_subscription(Int32 , '/second',self.subscribe_seconds,10)
        self.min_sub_ = self.create_subscription(Int32 , '/minute',self.subscribe_minutes,10)

        self.clock_pub_ = self.create_publisher(String, '/clock' ,10)
        self.timer = self.create_timer(1.0 , self.publish_seconds)


    def publish_seconds(self):
        msg = Int32()
        self.sec_= (self.sec_+1)%60
        msg.data = self.sec_
        self.sec_pub_.publish(msg)

        clock_msg = String()
        clock_msg.data = f"{self.hour_:02}:{self.min_:02}:{self.sec_:02}"
        self.clock_pub_.publish(clock_msg)
        self.get_logger().info(f'Time: {clock_msg.data}')

        

    def subscribe_seconds(self,sec:Int32):
        if sec.data == 59:
            msg = Int32()
            self.min_ = (self.min_+1)%60
            msg.data = self.min_
            self.min_pub_.publish(msg)
     

    def subscribe_minutes(self, min:Int32):
        if min.data == 59:
            msg = Int32()
            self.hour_ = (self.hour_+1)%24
            msg.data = self.hour_
            self.hour_pub_.publish(msg)
            self.get_logger().info(f'Hour Published: {msg.data}')
     


def main(args=None):
    rclpy.init(args=args)
    node = Clock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()