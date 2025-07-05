#!/usr/bin/env python3

# Import necessary ROS 2 libraries
import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

# Import custom action definition
from kratos_aditya_malviya.action import GoToAngle

# Import standard library
import time

class BonusServer(Node): 
    """
    A ROS2 Action Server that simulates rotating a robotic arm
    toward a given target angle by incrementing the current angle
    by 1° per second. Feedback is provided during the process,
    and a success message is returned upon completion.
    """

    def __init__(self):
        super().__init__("bonus_server") 

        # Initialize current angle
        self.cur_angle_ = 0
        
        # Create the Action Server
        self.bonus_server_ = ActionServer(
            self,
            GoToAngle,
            'bonus_server',
            execute_callback=self.execute_callback)
        
        self.get_logger().info("Action Server has been started")
        

    def execute_callback(self , goal_handle : ServerGoalHandle):
        """
        Executes the action when a goal is received.
        Increments the angle from current to target, publishing feedback.
        """

        # Extract the target angle from the goal request
        self.target_angle_= goal_handle.request.target_angle

        # Prepare feedback and result messages
        self.feedback_msg_ = GoToAngle.Feedback()
        self.result_ = GoToAngle.Result()

        self.get_logger().info("Executing")
        for _ in range(self.target_angle_):
            self.cur_angle_+=1

            # Publish feedback
            self.feedback_msg_.cur_angle = self.cur_angle_
            goal_handle.publish_feedback(self.feedback_msg_)
            self.get_logger().info(f"Current Angle: {self.cur_angle_}")

            time.sleep(1.0)
        
        # Mark goal as succeeded
        goal_handle.succeed()

        # Set result status
        self.result_.status = "Successfully Achieved"

        return self.result_


            

def main(args=None):
    """
    Main function to initialize and spin the action server node.
    """
    rclpy.init(args=args)
    node = BonusServer() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()