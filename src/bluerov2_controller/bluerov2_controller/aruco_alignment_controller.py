#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16
from scipy.spatial.transform import Rotation as R
import math
import time


class BlueROVController(Node):
    def __init__(self):
        super().__init__("aruco_controller")

        self.initialize_rc_defaults()
        self.initialize_publishers_and_subscribers()

        # Timestamp for the last received marker pose
        self.last_pose_time = time.time()
        self.marker_detected = False

        self.get_logger().info("BlueROV Controller Initialized.")

    def initialize_rc_defaults(self):
        """Initialize RC channel default values."""
        self.gripper = 1500
        self.pitch = 1500  # Unused for now
        self.throttle = 1500
        self.yaw = 1500
        self.forward = 1500
        self.lateral = 1500
        self.lights = 1100
        self.camera_tilt = 1500

    def initialize_publishers_and_subscribers(self):
        """Initialize ROS publishers and subscribers."""
        # Publishers
        self.forward_pub = self.create_publisher(UInt16, "/bluerov2/rc/ar/forward", 10)
        self.lateral_pub = self.create_publisher(UInt16, "/bluerov2/rc/ar/lateral", 10)
        self.yaw_pub = self.create_publisher(UInt16, "/bluerov2/rc/ar/yaw", 10)

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, "/aruco_pose", self.pose_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/bluerov2/imu/data", self.imu_callback, 10)

        # IMU yaw initialization
        self.current_imu_yaw = 0.0

    def imu_callback(self, msg):
        """Callback to process IMU data and extract yaw."""
        quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        euler_angles = R.from_quat(quaternion).as_euler('xyz', degrees=False)
        self.current_imu_yaw = euler_angles[2]
        self.get_logger().info(f"IMU Yaw: {math.degrees(self.current_imu_yaw):.2f}°")

    def pose_callback(self, msg):
        """Callback to handle ArUco pose data."""
        self.last_pose_time = time.time()
        self.marker_detected = True
        forward_Offset_correction = 0.7
        yaw_Offset_correction = -10
        lateral_Offset_correction = 0.4

        forward_distance = msg.pose.position.z
        lateral_offset = msg.pose.position.x - lateral_Offset_correction
        lateral_offset_yaw = msg.pose.position.x
        vertical_offset = -msg.pose.position.y

        if not hasattr(self, "yaw_integral_error"):
            self.initialize_pid_errors()

        # Calculate target yaw and yaw error
        # Offset correction (if needed)
        offset_angle = math.radians(yaw_Offset_correction)  # Adjust offset to observed error
        target_yaw = math.atan2(lateral_offset_yaw, forward_distance) - offset_angle
        yaw_error = self.normalize_angle(target_yaw - self.current_imu_yaw)

        # PID control for yaw
        yaw = self.calculate_pid(yaw_error, "yaw", kp=40, kd=20, base_pwm=1500)

        # Forward PID control
        forward_error = forward_distance - forward_Offset_correction
        forward = self.calculate_pid(forward_error, "forward", kp=79, kd=30, base_pwm=1500)

        # Lateral PID control
        lateral = self.calculate_pid(lateral_offset, "lateral", kp=30, kd=5, base_pwm=1500)

        # Vertical PID control
        throttle = self.calculate_pid(vertical_offset, "vertical", kp=200, kd=30, base_pwm=1500)

        # Publish control signals
        self.forward_pub.publish(UInt16(data=forward))
        self.lateral_pub.publish(UInt16(data=lateral))
        self.yaw_pub.publish(UInt16(data=yaw))


    def calculate_pid(self, error, pid_type, kp, kd, base_pwm):
        """Calculate PID control output."""
        integral_key = f"{pid_type}_integral_error"
        prev_error_key = f"prev_{pid_type}_error"

        integral_error = getattr(self, integral_key)
        integral_error += error
        delta_error = error - getattr(self, prev_error_key, 0.0)

        # PID formula
        control = kp * error + kd * delta_error
        pwm = self.clamp_rc_value(base_pwm + int(control))

        # Update errors
        setattr(self, integral_key, integral_error)
        setattr(self, prev_error_key, error)

        return pwm

    def initialize_pid_errors(self):
        """Initialize PID error attributes."""
        self.yaw_integral_error = 0.0
        self.forward_integral_error = 0.0
        self.lateral_integral_error = 0.0
        self.vertical_integral_error = 0.0

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def clamp_rc_value(self, value):
        """Clamp RC channel values to valid MAVLink range."""
        return max(1100, min(1900, int(value)))

    def stop_movement(self):
        """Stop all movements by setting RC values to neutral."""
        self.forward_pub.publish(UInt16(data=1500))
        self.lateral_pub.publish(UInt16(data=1500))
        self.yaw_pub.publish(UInt16(data=1500))

    def monitor_marker(self):
        """Monitor marker detection and stop movement if marker is lost."""
        current_time = time.time()
        if current_time - self.last_pose_time > 1.0:  # 1 second timeout
            if self.marker_detected:
                self.get_logger().warning("Marker lost. Stopping movement.")
                self.marker_detected = False
                self.initialize_pid_errors()
                self.stop_movement()


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVController()

    def timer_callback():
        node.monitor_marker()

    node.create_timer(0.1, timer_callback)  # Check for marker timeout every 100ms

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
