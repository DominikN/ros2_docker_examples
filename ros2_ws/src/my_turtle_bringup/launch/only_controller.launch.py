from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    move_controller_node = Node(
        package="my_demo_pkg",
        executable="move_controller"
    )

    color_controller_node = Node(
        package="my_demo_pkg",
        executable="color_controller"
    )

    ld.add_action(move_controller_node)
    ld.add_action(color_controller_node)
    return ld
