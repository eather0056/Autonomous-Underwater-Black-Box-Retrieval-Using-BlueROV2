from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
# from launch.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import FrontendLaunchDescriptionSource

def generate_launch_description():
    # Set the file paths
    mavros_launch_file = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'node.launch'
    )
    # joy_config_file = os.path.join(
    #     get_package_share_directory('autonomous_rov'), 'config', 'xbox.config.yaml'
    # )

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='bluerov2')
    fcu_url_arg = DeclareLaunchArgument('fcu_url', default_value='udp://192.168.2.1:14560@')
    gcs_url_arg = DeclareLaunchArgument('gcs_url', default_value='')
    tgt_system_arg = DeclareLaunchArgument('tgt_system', default_value='1')
    tgt_component_arg = DeclareLaunchArgument('tgt_component', default_value='1')
    log_output_arg = DeclareLaunchArgument('log_output', default_value='screen')
    fcu_protocol_arg = DeclareLaunchArgument('fcu_protocol', default_value='v2.0')
    respawn_mavros_arg = DeclareLaunchArgument('respawn_mavros', default_value='false')
    # joy_dev_arg = DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0')
    
    mavros_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(mavros_launch_file),
        launch_arguments={
            'pluginlists_yaml': os.path.join(get_package_share_directory('mavros'), 'launch', 'px4_pluginlists.yaml'),
            'config_yaml': os.path.join(get_package_share_directory('mavros'), 'launch', 'px4_config.yaml'),
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('tgt_system'),
            'tgt_component': LaunchConfiguration('tgt_component'),
            'log_output': LaunchConfiguration('log_output'),
            'fcu_protocol': LaunchConfiguration('fcu_protocol'),
            'respawn_mavros': LaunchConfiguration('respawn_mavros'),
            'namespace': LaunchConfiguration('namespace'),
        }.items()
    )
    # Nodes
    # joy_node = Node(
    #     package='joy',
    #     executable='joy_node',
    #     name='joy_node',
    #     namespace=LaunchConfiguration('namespace'),
    #     output='screen',
    #     parameters=[{'dev': LaunchConfiguration('joy_dev'), 'deadzone': 0.2, 'autorepeat_rate': 0.0}]
    # )

    # teleop_twist_joy_node = Node(
    #     package='teleop_twist_joy',
    #     executable='teleop_node',
    #     name='teleop_twist_joy_node',
    #     namespace=LaunchConfiguration('namespace'),
    #     output='screen',
    #     parameters=[
    #         {'require_enable_button': False, 'axis_linear.x': 1, 'axis_linear.y': 0, 'axis_linear.z': 4,
    #          'axis_angular.yaw': 3, 'axis_angular.roll': 7, 'axis_angular.pitch': 6,
    #          'scale_linear.x': 0.7, 'scale_linear.y': 0.7, 'scale_linear.z': 0.7,
    #          'scale_angular.yaw': 0.4, 'scale_angular.roll': 0.2, 'scale_angular.pitch': 0.2}
    #     ]
    # )

    # video_node = Node(
    #     package='autonomous_rov',
    #     executable='video',
    #     name='video_node',
    #     namespace=LaunchConfiguration('namespace'),
    #     output='screen'
    # )

    # listenerMIR_node = Node(
    #     package='autonomous_rov',
    #     executable='listenerMIR',
    #     name='listenerMIR',
    #     namespace=LaunchConfiguration('namespace'),
    #     output='screen'
    # )

    #  # MavlinkPressureNode
    # mavlink_pressure_node = Node(
    #     package='mavlink_pressure_node',  # Replace with your package name
    #     executable='pressure_node',  # Matches entry point in setup.py
    #     name='mavlink_pressure_node',
    #     parameters=[{'port': 14555}],  # Pass the UDP port parameter
    #     output='screen',
    # )
    
    # # Controllers
    # manual_controller_node = Node(
    #     package='autonomous_rov',
    #     executable='manual_controller',
    #     name='manual_controller',
    #     output='screen'
    # )

    # depth_controller_node = Node9

    # Return the LaunchDescription
    return LaunchDescription([
        namespace_arg,
        fcu_url_arg,
        gcs_url_arg,
        tgt_system_arg,
        tgt_component_arg,
        log_output_arg,
        fcu_protocol_arg,
        respawn_mavros_arg,
        # joy_dev_arg,
        mavros_launch,
        # joy_node,
        # teleop_twist_joy_node,
        # video_node,
        # mavlink_pressure_node,
        # manual_controller_node
        # # listenerMIR_node,
    ])
