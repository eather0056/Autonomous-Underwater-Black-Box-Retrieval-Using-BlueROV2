import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16

class ArucoForwardLateralController(Node):
    def __init__(self):
        super().__init__("aruco_forward_lateral_controller")

        # PID gains
        self.forward_kp = 79
        self.forward_ki = 0.0
        self.forward_kd = 30
        self.lateral_kp = 70
        self.lateral_ki = 0.0
        self.lateral_kd = 30

        # Desired setpoints
        self.desired_distance = 1.0  # Desired distance from the marker (meters)
        self.target_lateral = 0.0  # Target lateral alignment (centered)

        # Integral and previous error terms for PID
        self.forward_integral_error = 0.0
        self.lateral_integral_error = 0.0
        self.prev_forward_error = 0.0
        self.prev_lateral_error = 0.0

        # Create subscribers and publishers
        self.pose_sub = self.create_subscription(PoseStamped, "/aruco_pose", self.pose_callback, 10)
        self.forward_pub = self.create_publisher(UInt16, "/bluerov2/rc/ar/forward", 10)
        self.lateral_pub = self.create_publisher(UInt16, "/bluerov2/rc/ar/lateral", 10)

        self.get_logger().info("Aruco forward-lateral controller initialized successfully!")

    def pose_callback(self, msg):
        """Callback for processing ArUco pose data."""
        # Extract pose data
        forward_distance = msg.pose.position.z  # Distance to the marker (meters)
        lateral_offset = msg.pose.position.x    # Lateral offset from the marker (meters)

        # Compute forward PID control
        forward_error = forward_distance - self.desired_distance
        self.forward_integral_error += forward_error
        delta_forward_error = forward_error - self.prev_forward_error
        forward_control = (
            self.forward_kp * forward_error +
            self.forward_ki * self.forward_integral_error +
            self.forward_kd * delta_forward_error
        )
        self.prev_forward_error = forward_error

        # Compute lateral PID control
        lateral_error = lateral_offset - self.target_lateral
        self.lateral_integral_error += lateral_error
        delta_lateral_error = lateral_error - self.prev_lateral_error
        lateral_control = (
            self.lateral_kp * lateral_error +
            self.lateral_ki * self.lateral_integral_error +
            self.lateral_kd * delta_lateral_error
        )
        self.prev_lateral_error = lateral_error

        # Convert control outputs to PWM signals
        forward_pwm = self.rc_value(1500 + int(forward_control))
        lateral_pwm = self.rc_value(1500 + int(lateral_control))

        # Publish control signals
        self.forward_pub.publish(UInt16(data=forward_pwm))
        self.lateral_pub.publish(UInt16(data=lateral_pwm))

        # Debug logging
        self.get_logger().info(
            f"Aruco Control: Forward={forward_pwm}, Lateral={lateral_pwm} "
            f"(Forward Error={forward_error:.2f}, Lateral Error={lateral_error:.2f})"
        )

    def rc_value(self, value):
        """Clamp RC value to valid range [1100, 1900]."""
        return max(1100, min(1900, value))

def main(args=None):
    rclpy.init(args=args)
    node = ArucoForwardLateralController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
