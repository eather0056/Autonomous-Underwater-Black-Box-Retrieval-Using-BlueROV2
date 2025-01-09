#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math


class MarkerFollowerNode(Node):
    def __init__(self):
        super().__init__('marker_follower_node')

        # Parameters
        self.target_distance = 0.8  # Target distance from the marker in meters
        self.alignment_tolerance = 0.1  # Tolerance for alignment in meters
        self.linear_speed = 0.2    # Linear speed scaling
        self.alignment_speed = 0.1  # Speed for alignment (up/down and side-to-side)

        # Subscribe to the marker's pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_pose',  # Topic providing ArUco pose in the camera frame
            self.pose_callback,
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/bluerov1/cmd_vel', 10)

        self.get_logger().info("Marker Follower Node Initialized.")

    def pose_callback(self, msg):
        # Extract marker pose in the camera frame
        forward_distance = msg.pose.position.z  # Forward/backward motion
        vertical_offset = msg.pose.position.y   # Up/down motion
        lateral_offset = msg.pose.position.x    # Side-to-side motion

        # Initialize velocity command
        twist_msg = Twist()

        # Forward/backward control to maintain the target distance
        if abs(forward_distance - self.target_distance) > self.alignment_tolerance:
            # Move forward if too far, backward if too close
            twist_msg.linear.x = self.linear_speed * (1 if forward_distance > self.target_distance else -1)
        else:
            twist_msg.linear.x = 0.0  # Stop forward/backward motion

        # Up/down alignment based on vertical offset
        if abs(vertical_offset) > self.alignment_tolerance:
            # Move up or down to align vertically
            twist_msg.linear.y = self.alignment_speed * (-1 if vertical_offset > 0 else 1)
        else:
            twist_msg.linear.y = 0.0  # Stop up/down motion

        # Side-to-side alignment based on lateral offset
        if abs(lateral_offset) > self.alignment_tolerance:
            # Move left or right to align laterally
            twist_msg.linear.z = self.alignment_speed * (-1 if lateral_offset > 0 else 1)
        else:
            twist_msg.linear.z = 0.0  # Stop lateral motion

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist_msg)

        # Log the information for debugging
        self.get_logger().info(
            f"Forward Distance: {forward_distance:.2f} m, Vertical Offset: {vertical_offset:.2f} m, "
            f"Lateral Offset: {lateral_offset:.2f} m\n"
            f"Commanded Twist: linear.x={twist_msg.linear.x:.2f}, linear.y={twist_msg.linear.y:.2f}, "
            f"linear.z={twist_msg.linear.z:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Marker Follower Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
