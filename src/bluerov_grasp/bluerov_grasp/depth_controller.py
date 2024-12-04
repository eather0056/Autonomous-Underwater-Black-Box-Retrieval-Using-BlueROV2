import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Joy, Imu
from std_msgs.msg import Float64
from custom_msgs.msg import OverrideRCInput
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from scipy.spatial.transform import Rotation as R


class DepthControllerNode(Node):
    def __init__(self):
        super().__init__('depth_controller')

        # PID parameters
        self.depth_Kp = 1.5
        self.depth_Ki = 0
        self.depth_Kd = 0.2

        # Depth control variables
        self.depth_target = 1.0  # Desired depth in meters
        self.depth_integral = 0.0
        self.depth_prev_error = 0.0
        self.depth_last_time = None
        self.Correction_depth = 1500  # Default PWM value (neutral position)
        self.prev_btn_depth_hold = 0

        self.set_mode = [True, False, False]

        # Pressure variables
        self.latest_pressure = None
        self.depth_p0 = None
        self.init_p0 = True

        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 depth=1)
        # Publishers and Subscribers
        self.pub_depth = self.create_publisher(Float64, '/depth', 10)
        self.pub_rcinn = self.create_publisher(OverrideRCInput, '/rc_input', 10)
        self.sub_pressure = self.create_subscription(FluidPressure, '/bluerov2/scaled_pressure2', self.PressureCallback, 10)
        self.sub_joy = self.create_subscription(Joy, '/joyy', self.joyCallback, 10)
        self.target_depth = self.create_subscription(Float64, "/bluerov2/depth_desired", self.desiredDepthCallback ,qos_profile)

        # For YAW
        self.yaw_subscription = self.create_subscription(
            Imu,
            'bluerov2/imu/data',
            self.imu_callback,
            qos_profile)
        self.target_yaw_sub = self.create_subscription(Float64, '/yaw_target', self.desireYawCallback, 10)
        self.kp_yaw = 0.5 # Proportional gain
        self.ki_yaw = 0  # Integral gain
        self.kd_yaw = 0.05 # Derivative gain

        self.prev_error_yaw = 0.0
        self.integral_yaw = 0.0
        self.prev_time_yaw = None

        self.desired_yaw = 1.57  # Example: keep the yaw at 0 (facing forward)
        self.current_yaw = None

        self.get_logger().info('Depth and Yaw Controller Node Initialized')

    
    def desireYawCallback(self, data):
        # Only in AUTOMATIC Mode
        if self.set_mode[1]:
            self.desired_yaw = data

    def reset_variables(self):
        self.depth_integral = 0
        self.depth_last_time = None
        self.depth_prev_error = 0
        
        self.prev_error_yaw = 0.0
        self.integral_yaw = 0.0
        self.prev_time_yaw = None

    def desiredDepthCallback(self, data):
        # Only use subscribed depth if AUTOMATIC Mode
        if self.set_mode[1]:
            self.depth_target = data

    def joyCallback(self, data):
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_depth_hold_mode = data.buttons[1]

        if btn_depth_hold_mode and not self.prev_btn_depth_hold:
            self.set_mode = [False, False, True]
            rho = 1000.0  # Density of water in kg/m^3
            g = 9.80665   # Gravity in m/s^2
            current_pressure = self.latest_pressure  # Store the latest pressure in the callback
            current_depth = (current_pressure - 101300) / (rho * g) - self.depth_p0
            self.depth_target = current_depth
            # self.depth_hold_enabled = True
            self.get_logger().info(f"Depth target set to: {self.depth_target}")

            self.desired_yaw = self.current_yaw
            self.get_logger().info(f"Yaw Desired set to: {self.current_yaw}")
        self.prev_btn_depth_hold = btn_depth_hold_mode

        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False, False]
            self.reset_variables()
            self.get_logger().info("Mode manual")
        if btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True, False]
            self.reset_variables()
            self.get_logger().info("Mode automatic")
        if btn_depth_hold_mode and not self.set_mode[2]:
            # self.init_a0 = self.init_p0 = True
            self.set_mode = [False, False, True]
            self.reset_variables()
            self.get_logger().info("Mode Deph Hold")
    
    def imu_callback(self, msg: Imu):
        # Extract the yaw angle from the quaternion
        q = msg.orientation
        # roll, pitch, yaw = euler_from_quaternion(q)
        quaternion = [
        q.x,
        q.y,
        q.z,
        q.w,
         ]
        rotation = R.from_quat(quaternion)
        roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)

        # Compute the control signal
        self.current_yaw = yaw
        print(self.current_yaw)

        if self.set_mode[1]:
            control_signal = self.pid_control(yaw)

        # # Publish the control signal to the thruster topic
        # self.send_thruster_command(control_signal)

    # def quaternion_to_euler_yaw(self, x, y, z, w):
    #     """Convert quaternion to yaw (in radians)."""
    #     siny_cosp = 2.0 * (w * z + x * y)
    #     cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    #     yaw = math.atan2(siny_cosp, cosy_cosp)
    #     return yaw

    def pid_control(self, current_yaw):
        """Calculate the PID control signal."""
        error = self.desired_yaw - current_yaw
        # Ensure error is within [-pi, pi]
        error = (error + math.pi) % (2 * math.pi) - math.pi

        print(error)

        # Calculate time step
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.prev_time_yaw if self.prev_time_yaw else 0.0
        self.prev_time_yaw = current_time

        if dt > 0:
            # Proportional term
            proportional = self.kp_yaw * error

            # Integral term
            self.integral_yaw += error * dt
            integral = self.ki_yaw * self.integral_yaw

            # Derivative term
            derivative = self.kd_yaw * (error - self.prev_error_yaw) / dt
            self.prev_error_yaw = error

            # Total control signal
            control_signal = proportional + integral + derivative
            pwm_control = self.mapValueScalSat(-control_signal)

            print(pwm_control)

            rcinn_msg = OverrideRCInput()
            rcinn_msg.pitch = 0
            rcinn_msg.roll = 0
            rcinn_msg.throttle = 0
            rcinn_msg.yaw = pwm_control
            rcinn_msg.forward = 0
            rcinn_msg.lateral = 0
            return self.pub_rcinn.publish(rcinn_msg)
    
    def mapValueScalSat(self, value):
        # Scale joystick input (-1 to 1) to PWM range (1100-1900) with neutral at 1500
        pulse_width = value * 400 + 1500
        # Saturate values within range
        pulse_width = min(max(pulse_width, 1350), 1650)
        return int(pulse_width)

    def PressureCallback(self, data): 
        print(f"Depth Target set to : {self.depth_target} m")
        pressure = data.fluid_pressure
        self.latest_pressure = pressure
        rho = 1000.0  # Density of water in kg/m^3
        g = 9.80665   # Gravity in m/s^2

        # Initialize reference depth (p0)
        if self.init_p0:
            self.depth_p0 = (pressure - 101300) / (rho * g)
            self.init_p0 = False

        # Calculate current depth
        depth = (pressure - 101300) / (rho * g) - self.depth_p0

        # Publish depth for monitoring
        depth_msg = Float64()
        depth_msg.data = depth
        self.pub_depth.publish(depth_msg)
         
        # Perform depth control if Automatic Mode
        if self.set_mode[1] or self.set_mode[2]:
            depth_error = self.depth_target - depth
            print(depth_error)
            current_time = self.get_clock().now().nanoseconds * 1e-9  # Convert to seconds

            if self.depth_last_time is None:
                dt = 0.0
            else:
                dt = current_time - self.depth_last_time

            self.depth_last_time = current_time

            # PID control logic
            if dt > 0.0:
                derivative = (depth_error - self.depth_prev_error) / dt
            else:
                derivative = 0.0

            self.depth_integral += depth_error * dt

            control_signal = (self.depth_Kp * depth_error +
                              self.depth_Ki * self.depth_integral +
                              self.depth_Kd * derivative)
            print(control_signal)
            self.depth_prev_error = depth_error

            # Map control signal to PWM
            self.Correction_depth = self.mapValueScalSat(-control_signal)

            # Publish RCINN signal
            rcinn_msg = OverrideRCInput()
            rcinn_msg.pitch = 0
            rcinn_msg.roll = 0
            rcinn_msg.throttle = self.Correction_depth
            rcinn_msg.yaw = 0
            rcinn_msg.forward = 0
            rcinn_msg.lateral = 0
            self.pub_rcinn.publish(rcinn_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
