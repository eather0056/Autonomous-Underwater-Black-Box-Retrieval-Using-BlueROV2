#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    video_node = Node(
        package="autonomous_rov",
        executable="video",
    )

    ld.add_action(video_node)

    return ld
