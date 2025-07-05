#!/usr/bin/env python3

# Import ROS 2 client libraries
import rclpy
from rclpy.node import Node

# Import standard message types
from std_msgs.msg import String, Int32


class Clock(Node):
    """
    A ROS2 node that simulates a digital clock.
    It publishes time in HH:MM:SS format to the /clock topic,
    and separately publishes seconds, minutes, and hours
    on /second, /minute, and /hour topics respectively.
    """

    def __init__(self):
        super().__init__("q4_clock")

        # Initialize clock counters
        self.sec_ = 0
        self.min_ = 0
        self.hour_ = 0

        # Publishers for seconds, minutes, and hours
        self.sec_pub_ = self.create_publisher(Int32, '/second', 10)
        self.min_pub_ = self.create_publisher(Int32, '/minute', 10)
        self.hour_pub_ = self.create_publisher(Int32, '/hour', 10)

        # Subscribers to handle second and minute rollovers
        self.sec_sub_ = self.create_subscription(Int32, '/second', self.subscribe_seconds, 10)
        self.min_sub_ = self.create_subscription(Int32, '/minute', self.subscribe_minutes, 10)

        # Publisher for full clock time in HH:MM:SS string format
        self.clock_pub_ = self.create_publisher(String, '/clock', 10)

        # Timer that triggers every second to simulate ticking
        self.timer = self.create_timer(1.0, self.publish_seconds)

    def publish_seconds(self):
        """
        Callback function for the timer.
        Increments seconds, wraps at 60, publishes to /second,
        and also publishes the full clock time to /clock.
        """
        # Increment and wrap seconds
        self.sec_ = (self.sec_ + 1) % 60

        # Publish seconds to /second
        sec_msg = Int32()
        sec_msg.data = self.sec_
        self.sec_pub_.publish(sec_msg)

        # Format and publish complete time to /clock
        clock_msg = String()
        clock_msg.data = f"{self.hour_:02}:{self.min_:02}:{self.sec_:02}"
        self.clock_pub_.publish(clock_msg)

        # Log the current time
        self.get_logger().info(f'Time: {clock_msg.data}')

    def subscribe_seconds(self, sec: Int32):
        """
        Callback when a new second is received.
        If seconds wrap to 59, increment minutes and publish to /minute.
        """
        if sec.data == 59:
            self.min_ = (self.min_ + 1) % 60

            min_msg = Int32()
            min_msg.data = self.min_
            self.min_pub_.publish(min_msg)

    def subscribe_minutes(self, min: Int32):
        """
        Callback when a new minute is received.
        If minutes wrap to 59, increment hours and publish to /hour.
        """
        if min.data == 59:
            self.hour_ = (self.hour_ + 1) % 24

            hour_msg = Int32()
            hour_msg.data = self.hour_
            self.hour_pub_.publish(hour_msg)


def main(args=None):
    """
    Main function to initialize and spin the Clock node.
    """
    rclpy.init(args=args)
    node = Clock()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
