#!/usr/bin/env python3

import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from kratos_aditya_malviya.action import GoToAngle
import time

class BonusServer(Node): 

    def __init__(self):
        super().__init__("bonus_server") 
        self.cur_angle_ = 0
        
        self.bonus_server_ = ActionServer(
            self,
            GoToAngle,
            'bonus_server',
            execute_callback=self.execute_callback)
        
        self.get_logger().info("Action Server has been started")
        

    def execute_callback(self , goal_handle : ServerGoalHandle):
        self.target_angle_= goal_handle.request.target_angle

        self.feedback_msg_ = GoToAngle.Feedback()
        self.result_ = GoToAngle.Result()

        self.get_logger().info("Executing")
        for _ in range(self.target_angle_):
            self.cur_angle_+=1
            self.feedback_msg_.cur_angle = self.cur_angle_
            goal_handle.publish_feedback(self.feedback_msg_)
            self.get_logger().info(f"Current Angle: {self.cur_angle_}")
            time.sleep(1.0)
        
        goal_handle.succeed()
        self.result_.status = "Successfully Achieved"
        return self.result_


            

def main(args=None):
    rclpy.init(args=args)
    node = BonusServer() 
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()