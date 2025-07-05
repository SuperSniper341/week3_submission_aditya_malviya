#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from kratos_aditya_malviya.action import GoToAngle

class BonusClient(Node): 

    def __init__(self):
        super().__init__("bonus_client") 
        self.bonus_client_ = ActionClient(
            self,
            GoToAngle,
            "bonus_server"
        )
        
    def send_goal(self,target_angle):
        # Wait for the server
        self.bonus_client_.wait_for_server()

        # Create a goal
        goal = GoToAngle.Goal()
        goal.target_angle = target_angle

        # Send the goal
        self.bonus_client_.send_goal_async(goal,self.feedback_callback).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self , future):
        self.goal_handle_ : ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result.status}")

    def feedback_callback(self, feedback_msg):
        feedback : GoToAngle.Feedback = feedback_msg.feedback
        cur_angle = feedback.cur_angle
        self.get_logger().info(f"Current Angle: {cur_angle}")
    


def main(args=None):
    rclpy.init(args=args)
    node = BonusClient() 
    node.send_goal(9)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()