#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    controller_node = Node(
        package="bluerov2_controller",
        executable="controller_ld",
    )

    video_node = Node(
        package="bluerov2_controller",
        executable="video",
    )

    input_node = Node(
        package="bluerov2_controller",
        executable="input_controller_ld",
    )

    depth_node = Node(
        package="bluerov2_controller",
        executable="depth_controller",
    )

    yaw_node = Node(
        package="bluerov2_controller",
        executable="aruco_yaw_controller",
    ) 

    aruco_alignment_node = Node(
        package="bluerov2_controller",
        executable="aruco_alignment_controller",
        parameters=[{
            'target_x': 1.5,
            'target_y': 0.0,
            'kp': 600,
            'ki': 100,
            'kd': 50,
            'enable': True
        }]
    )
    gui_input_node = Node(
        package="bluerov2_controller",
        executable="gui_controller",
    )

    ld.add_action(controller_node)    
    ld.add_action(depth_node)
    #ld.add_action(yaw_node)
    ld.add_action(video_node)
    ld.add_action(input_node)
    ld.add_action(aruco_alignment_node)
    #ld.add_action(gui_input_node)
    return ld