from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bluerov_grasp',
            executable='marker_detection_node',
            name='marker_detection_node',
            output='screen',
            parameters=[
                {'camera_topic': '/camera/image_raw'},
                {'camera_info_topic': '/camera/camera_info'},
                {'marker_length': 0.1},
                {'aruco_dict': 'DICT_4X4_50'},
                {'handle_offset_x': 0.05},
                {'handle_offset_y': 0.0},
                {'handle_offset_z': 0.0},
            ]
        ),
        Node(
            package='bluerov_grasp',
            executable='grasp_controller_node',
            name='grasp_controller_node',
            output='screen',
            parameters=[
                {'handle_pose_topic': '/handle_pose'},
                {'cmd_vel_topic': '/cmd_vel'},
                {'gripper_command_topic': '/gripper/command'},
                {'grasping_distance_threshold': 0.2},
                {'kp': 0.5},
            ]
        )
    ])
