from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="meeseeks",
            executable="robot_gestures",
            name="robot_gestures",
            output="screen",
        ),
        Node(
            package="meeseeks",
            executable="main_logic",
            name="main_logic",
            output="screen",
        ),
    ])
