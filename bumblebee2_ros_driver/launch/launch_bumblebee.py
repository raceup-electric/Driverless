from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="bumblebee2_ros_driver",
            executable="test_bumblebee2",
            output="screen",
        )
    ])