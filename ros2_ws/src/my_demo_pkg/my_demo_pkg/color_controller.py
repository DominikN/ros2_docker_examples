#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial

from turtlesim.srv import SetPen
import random

class ColorControllerNode(Node):
    def __init__(self):
        super().__init__("color_controller")
        self.r_ = 100
        self.g_ = 100
        self.b_ = 100
        self.control_loop_timer_ = self.create_timer(1.0, self.control_loop)

    def control_loop(self):
        self.r_ = random.randint(0, 255)
        self.g_ = random.randint(0, 255)
        self.b_ = random.randint(0, 255)
        self.call_set_pen_server(self.r_, self.g_, self.b_)

    def call_set_pen_server(self, r, g, b):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for server Set Pen....")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = 10

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen, r=r, g=g, b=b))
        
    def callback_set_pen(self, future, r, g, b):
        try:
            response = future.result()
            self.get_logger().info("[" +str(r) + ", " + str(g) + ", " + str(b) + "] SetPen srv response: " + str(response))
        
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = ColorControllerNode()
    rclpy.spin(node)
    rclpy.shutdonw()

if __name__ == "__main__":
    main()