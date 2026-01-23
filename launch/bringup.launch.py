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
            executable="robot_initialization",
            name="robot_initialization",
            output="screen",
        ),
        Node(
            package="meeseeks",
            executable="main_logic",
            name="main_logic",
            output="screen",
        ),
        Node(
            package="meeseeks",
            executable="pointing_to_target_logic",
            name="pointing_to_target",
            output="screen",
        ),
    ])
