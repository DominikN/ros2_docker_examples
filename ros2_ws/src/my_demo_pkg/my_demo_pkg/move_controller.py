#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class MoveControllerNode(Node): 
    def __init__(self):
        super().__init__("move_controller") 

        self.cmd_vel_publisher_ = self.create_publisher(
        Twist, "turtle1/cmd_vel", 10)

        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    def control_loop(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z =0.3*math.pi

        # self.get_logger().info("publishing to turtle1/cmd_vel")
        self.cmd_vel_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveControllerNode() 
    rclpy.spin(node)
    rclpy.shutdonw()

if __name__ == "__main__":
    main()