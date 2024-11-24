#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
import math

class FollowMarkerNode(Node):
    def __init__(self):
        super().__init__('follow_marker_node')

        # Parameters
        self.target_distance = 0.8  # Target distance from the marker in meters
        self.linear_speed = 0.2    # Linear speed in m/s
        self.angular_speed = 0.5   # Angular speed in rad/s

        # Subscription
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_pose',  # ArUco marker pose topic in the camera frame
            self.pose_callback,
            10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/bluerov1/cmd_vel', 10)

        self.get_logger().info("Follow Marker Node Initialized.")

    def pose_callback(self, msg):
        # Marker pose in camera frame
        marker_x = msg.pose.position.z  # Forward distance (in meters)
        marker_y = msg.pose.position.x  # Lateral offset (in meters)
        marker_z = msg.pose.position.y  # Vertical offset (in meters)

        # Calculate distance to the marker
        distance = math.sqrt(marker_x**2 + marker_y**2 + marker_z**2)

        # Calculate control commands
        twist_msg = Twist()

        # Forward/backward to maintain target distance
        if marker_x > self.target_distance:
            twist_msg.linear.x = self.linear_speed
        elif marker_x < self.target_distance:
            twist_msg.linear.x = -self.linear_speed
        else:
            twist_msg.linear.x = 0.0

        # Lateral and vertical adjustments
        twist_msg.linear.y = -self.linear_speed * marker_y  # Adjust lateral movement
        twist_msg.linear.z = -self.linear_speed * marker_z  # Adjust vertical movement

        # Rotate robot to face the marker
        yaw_correction = math.atan2(marker_y, marker_x)  # Calculate yaw angle in camera frame
        twist_msg.angular.z = -self.angular_speed * yaw_correction

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist_msg)

        # Logging for debugging
        self.get_logger().info(
            f"Published Twist: linear.x={twist_msg.linear.x}, linear.y={twist_msg.linear.y}, "
            f"linear.z={twist_msg.linear.z}, angular.z={twist_msg.angular.z}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Follow Marker Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
