import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt16, Float64
from scipy.spatial.transform import Rotation as R
import math
import time

class BlueROVController(Node):
    def __init__(self):
        super().__init__("aruco_controller")

        self.initialize_rc_defaults()
        self.initialize_publishers_and_subscribers()
        self.initialize_parameters()

        # Timestamp for the last received marker pose
        self.last_pose_time = time.time()
        self.marker_detected = False
        self.manual_override = False

        # Track last forward distance and yaw error
        self.last_forward_distance = float('inf')
        self.last_yaw_error = 0.0

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
        self.gripper_pub = self.create_publisher(UInt16, "/bluerov2/rc/ar/gripper", 10)

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, "/aruco_pose", self.pose_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/bluerov2/imu/data", self.imu_callback, 10)

        # Parameters from topics
        self.create_subscription(Float64, "/settings/yaw/kp", lambda msg: self.update_param("yaw_kp", msg.data), 10)
        self.create_subscription(Float64, "/settings/yaw/kd", lambda msg: self.update_param("yaw_kd", msg.data), 10)
        self.create_subscription(Float64, "/settings/forward/kp", lambda msg: self.update_param("forward_kp", msg.data), 10)
        self.create_subscription(Float64, "/settings/forward/kd", lambda msg: self.update_param("forward_kd", msg.data), 10)
        self.create_subscription(Float64, "/setings/forward_Offset_correction", lambda msg: self.update_param("forward_Offset_correction", msg.data), 10)

        # IMU yaw initialization
        self.current_imu_yaw = 0.0

    def initialize_parameters(self):
        """Initialize default parameters."""
        self.yaw_kp = 40
        self.yaw_kd = 20
        self.forward_kp = 79
        self.forward_kd = 30
        self.forward_Offset_correction = 0.7

    def update_param(self, param_name, value):
        """Update parameter value dynamically."""
        setattr(self, param_name, value)
        self.get_logger().info(f"Updated {param_name} to {value}")

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

        forward_distance = msg.pose.position.z
        lateral_offset = msg.pose.position.x

        self.last_forward_distance = forward_distance

        if not hasattr(self, "yaw_integral_error"):
            self.initialize_pid_errors()

        # PID control for yaw
        yaw_error = self.normalize_angle(math.atan2(lateral_offset, forward_distance) - self.current_imu_yaw)
        self.last_yaw_error = yaw_error
        yaw = self.calculate_pid(yaw_error, "yaw", kp=self.yaw_kp, kd=self.yaw_kd, base_pwm=1500)

        # Forward PID control
        forward_error = forward_distance - self.forward_Offset_correction
        forward = self.calculate_pid(forward_error, "forward", kp=self.forward_kp, kd=self.forward_kd, base_pwm=1500)

        # Publish control signals
        self.forward_pub.publish(UInt16(data=forward))
        self.yaw_pub.publish(UInt16(data=yaw))

        # Open gripper if close enough to the target
        if forward_distance <= 0.5:
            self.grab_handle()

        self.get_logger().info(f"Aligning: Forward={forward}, Yaw={yaw}, Distance={forward_distance:.2f}m")

    def calculate_pid(self, error, pid_type, kp, kd, base_pwm):
        """Calculate PID control output."""
        integral_key = f"{pid_type}_integral_error"
        prev_error_key = f"prev_{pid_type}_error"

        integral_error = getattr(self, integral_key, 0.0)
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

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def clamp_rc_value(self, value):
        """Clamp RC channel values to valid MAVLink range."""
        return max(1100, min(1900, int(value)))

    def stop_movement(self):
        """Stop all movements by setting RC values to neutral."""
        self.forward_pub.publish(UInt16(data=1500))
        self.yaw_pub.publish(UInt16(data=1500))

    def monitor_marker(self):
        """Monitor marker detection and stop movement if marker is lost."""
        current_time = time.time()
        if current_time - self.last_pose_time > 1.0:  # 1 second timeout
            if self.marker_detected:
                self.get_logger().warning("Marker lost. Switching to approximate approach.")
                self.marker_detected = False
                self.manual_override = True
                self.approach_target()

    def approach_target(self):
        """Approach the target when marker is lost."""
        if self.last_forward_distance < 0.4:  # Only move forward if within a safe range
            self.forward_pub.publish(UInt16(data=1600))
            self.yaw_pub.publish(UInt16(data=self.calculate_pid(self.last_yaw_error, "yaw", kp=self.yaw_kp, kd=self.yaw_kd, base_pwm=1500)))
            self.get_logger().info("Approaching target in manual mode with yaw holding.")
        else:
            self.get_logger().info("Target too far for safe approach. Holding position.")

    def grab_handle(self):
        """Activate the gripper to grab the handle."""
        self.gripper_pub.publish(UInt16(data=1900))
        self.get_logger().info("Gripper activated.")


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
