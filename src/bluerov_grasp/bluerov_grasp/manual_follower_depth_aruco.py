#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import pygame
from pygame.locals import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Bool, Header, Float64

class BlueROVController(Node):
    def __init__(self):
        super().__init__("bluerov_controller")
        self.initialize_mavlink_config()
        self.initialize_rc_defaults()
        self.initialize_ros_publishers()
        self.initialize_mavlink_connection()
        self.initialize_joystick()

        # Start disarmed and in manual mode
        self.armed = False
        self.mode = "manual"  # Modes: "manual", "aruco", "depth"

        # Depth PID parameters
        self.depth_Kp = 1.0    # Adjusted Proportional gain
        self.depth_Ki = 0.1    # Introduced Integral gain
        self.depth_Kd = 0.3    # Increased Derivative gain

        # Depth control variables
        self.depth_target = 0.0  # Desired depth in meters
        self.depth_integral = 0.0
        self.depth_prev_error = 0.0
        self.depth_last_time = None
        self.depth_p0 = None
        self.current_depth = 0.0
        self.init_p0 = True  # For initializing reference depth

        # Autonomous mode gains
        self.forward_gain = 50
        self.lateral_gain = 50
        self.vertical_gain = 50

        # Subscribe to ArUco pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped, '/aruco_pose', self.pose_callback, 10)

        # Start update loops
        self.start_heartbeat_and_input_loops()
        self.get_logger().info("BlueROV Controller Initialized.")

    ### Initialization Functions ###
    def initialize_mavlink_config(self):
        self.heartbeat_period = 1.0  # Heartbeat every 1 second

    def initialize_rc_defaults(self):
        self.gripper = 1500  # RC1
        self.pitch = 1500    # RC2 (Unused for now)
        self.throttle = 1500  # RC3
        self.yaw = 1500  # RC4
        self.forward = 1500  # RC5
        self.lateral = 1500  # RC6
        self.lights = 1500  # RC7
        self.camera_tilt = 1500  # RC8

    def initialize_ros_publishers(self):
        self.arm_pub = self.create_publisher(Bool, "/bluerov2/arm_status", 10)
        self.pressure_publisher = self.create_publisher(
            FluidPressure, '/bluerov2/scaled_pressure2', 10)
        self.depth_publisher = self.create_publisher(
            Float64, '/depth', 10)

    def initialize_mavlink_connection(self):
        self.declare_parameter("ip", "192.168.2.1")
        self.declare_parameter("port", 14555)

        self.bluerov_ip = self.get_parameter("ip").value
        self.bluerov_port = self.get_parameter("port").value

        self.get_logger().info("Connecting to BlueROV2...")
        self.connection = mavutil.mavlink_connection(
            f"udpin:{self.bluerov_ip}:{self.bluerov_port}")
        self.connection.wait_heartbeat()
        self.get_logger().info("BlueROV2 connection established.")

        self.mav = self.connection.mav
        self.target = (self.connection.target_system, self.connection.target_component)

    def initialize_joystick(self):
        pygame.init()
        self.joysticks = []
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.joysticks.append(joystick)
            self.get_logger().info(f"Joystick detected: {joystick.get_name()}")

    def start_heartbeat_and_input_loops(self):
        self.create_timer(0.02, self.update_inputs)  # Joystick input loop at 50 Hz
        self.create_timer(0.02, self.send_bluerov_commands)  # Sends commands to BlueROV
        self.create_timer(0.01, self.read_mavlink_messages)  # Read MAVLink messages
        self.create_timer(self.heartbeat_period, self.send_heartbeat)  # Send heartbeat

    ### MAVLink Command Functions ###
    def arm(self):
        if not self.armed:
            self.connection.arducopter_arm()
            self.connection.motors_armed_wait()
            self.armed = True
            self.get_logger().info("Thrusters armed!")

    def disarm(self):
        if self.armed:
            self.connection.arducopter_disarm()
            self.connection.motors_disarmed_wait()
            self.armed = False
            self.get_logger().info("Thrusters disarmed!")

    def clamp_rc_value(self, value):
        return max(1100, min(1900, int(value)))

    def map_value_scal_sat(self, value):
        """Scale joystick input (-1 to 1) to PWM range (1100-1900) with neutral at 1500."""
        return self.clamp_rc_value(value * 400 + 1500)

    def send_rc_override(self, forward, lateral, throttle, yaw):
        rc_channel_values = (
            self.clamp_rc_value(self.gripper),  # RC1
            1500,                               # RC2 (Unused for now)
            self.clamp_rc_value(throttle),      # RC3
            self.clamp_rc_value(yaw),           # RC4
            self.clamp_rc_value(forward),       # RC5
            self.clamp_rc_value(lateral),       # RC6
            self.clamp_rc_value(self.lights),   # RC7
            self.clamp_rc_value(self.camera_tilt),  # RC8
            65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535
        )
        self.mav.rc_channels_override_send(*self.target, *rc_channel_values)

    def send_bluerov_commands(self):
        """
        Sends commands to BlueROV based on the current mode.
        """
        if self.mode == "manual":
            self.send_rc_override(self.forward, self.lateral, self.throttle, self.yaw)
        elif self.mode == "aruco":
            # ArUco mode logic handled in pose_callback
            pass
        elif self.mode == "depth":
            # Perform depth control
            self.perform_depth_control()

    ### Heartbeat Function ###
    def send_heartbeat(self):
        self.mav.heartbeat_send(
            type=mavutil.mavlink.MAV_TYPE_GCS,
            autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=mavutil.mavlink.MAV_STATE_ACTIVE
        )

    ### Input Handling ###
    def update_inputs(self):
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                self.handle_joystick_motion(event)
            elif event.type == JOYBUTTONDOWN:
                self.handle_button_press(event)

    def handle_joystick_motion(self, event):
        value = event.value
        pwm_value = self.map_value_scal_sat(value)

        if self.mode == "manual":
            if event.axis == 0:
                self.lateral = pwm_value
            elif event.axis == 1:
                self.forward = self.map_value_scal_sat(-value)
            elif event.axis == 3:
                self.yaw = pwm_value
            elif event.axis == 4:
                self.throttle = pwm_value
            elif event.axis == 2:
                self.camera_tilt = pwm_value
        elif self.mode == "depth":
            # Allow yaw control in depth hold mode
            if event.axis == 3:
                self.yaw = pwm_value

    def handle_button_press(self, event):
        if event.button == 7:
            if self.armed:
                self.disarm()
            else:
                self.arm()
        elif event.button == 6:
            if self.mode == "manual":
                self.mode = "aruco"
            elif self.mode == "aruco":
                self.mode = "depth"
                # Set target depth to current depth
                self.depth_target = self.current_depth
                # Reset PID variables
                self.depth_integral = 0.0
                self.depth_prev_error = 0.0
                self.depth_last_time = None
                self.init_p0 = True  # Re-initialize reference depth
                self.get_logger().info(f"Depth hold activated at depth: {self.current_depth:.2f} m")
            else:
                self.mode = "manual"
            self.get_logger().info(f"Switched to {self.mode} mode.")
        elif event.button == 0:  # Button A: Close gripper
            self.gripper = 1900
        elif event.button == 1:  # Button B: Open gripper
            self.gripper = 1100
        elif event.button == 4:  # Left Bumper: Decrease light intensity
            self.lights = max(1100, self.lights - 100)
            self.get_logger().info(f"Light intensity decreased: {self.lights}")
        elif event.button == 5:  # Right Bumper: Increase light intensity
            self.lights = min(1900, self.lights + 100)
            self.get_logger().info(f"Light intensity increased: {self.lights}")

    ### ArUco Mode ###
    def pose_callback(self, msg):
        if self.mode != "aruco" or not self.armed:
            return

        forward_distance = msg.pose.position.z
        lateral_offset = msg.pose.position.x
        vertical_offset = msg.pose.position.y

        forward = 1500 + int((forward_distance - 1.5) * self.forward_gain)
        lateral = 1500 + int(lateral_offset * self.lateral_gain)
        throttle = 1500 + int(vertical_offset * self.vertical_gain)

        self.send_rc_override(forward, lateral, throttle, self.yaw)
        self.get_logger().info(f"Aruco: Forward={forward}, Lateral={lateral}, Throttle={throttle}")

    ### Depth Hold Mode ###
    def perform_depth_control(self):
        # Depth control logic
        depth_error = self.depth_target - self.current_depth

        current_time = self.get_clock().now().nanoseconds * 1e-9  # Convert to seconds

        if self.depth_last_time is None:
            dt = 0.02  # Default to control loop interval
        else:
            dt = current_time - self.depth_last_time
            if dt <= 0.0 or dt > 1.0:
                dt = 0.02  # Discard invalid dt values

        self.depth_last_time = current_time

        # PID control logic
        derivative = (depth_error - self.depth_prev_error) / dt

        # Limit derivative to prevent spikes due to noise
        max_derivative = 0.5  # Adjust based on observed noise levels
        derivative = max(min(derivative, max_derivative), -max_derivative)

        self.depth_integral += depth_error * dt

        # Implement windup guard
        max_integral = 1.0  # Prevent integral windup
        self.depth_integral = max(min(self.depth_integral, max_integral), -max_integral)

        control_signal = (self.depth_Kp * depth_error +
                          self.depth_Ki * self.depth_integral +
                          self.depth_Kd * derivative)

        self.depth_prev_error = depth_error

        # Map control signal to PWM
        pwm = self.map_value_scale_sat_depth(-control_signal)

        # Send RC override (only throttle and yaw)
        self.send_rc_override(self.forward, self.lateral, pwm, self.yaw)

        # Log for debugging
        self.get_logger().info(
            f"Depth Control: error={depth_error:.3f}, control_signal={control_signal:.2f}, "
            f"pwm={pwm}, current_depth={self.current_depth:.2f}")

    def map_value_scale_sat_depth(self, value):
        # Map control signal to PWM value with adjusted scaling and limits
        pulse_width = value * 300 + 1500  # Adjusted scaling factor
        pulse_width = max(1400, min(1600, pulse_width))  # Adjusted PWM limits
        return int(pulse_width)

    ### MAVLink Message Reading ###
    def read_mavlink_messages(self):
        try:
            message = self.connection.recv_match(blocking=False)
            if not message:
                return

            if message.get_type() == 'HEARTBEAT':
                # Handle heartbeat messages if necessary
                pass

            if message.get_type() == 'SCALED_PRESSURE2':
                pressure = message.press_abs  # Pressure in hPa

                # Convert hPa to Pa for consistency
                pressure_pa = pressure * 100.0

                # Create FluidPressure message (if needed)
                pressure_msg = FluidPressure()
                pressure_msg.header = Header()
                pressure_msg.header.stamp = self.get_clock().now().to_msg()
                pressure_msg.header.frame_id = 'base_link'
                pressure_msg.fluid_pressure = pressure_pa  # Pressure in Pa
                pressure_msg.variance = 0.0

                # Publish pressure message
                self.pressure_publisher.publish(pressure_msg)

                # Update current_depth
                surface_pressure_pa = 101300.0  # Standard atmospheric pressure at sea level in Pa
                water_density = 1000.0  # Density of water in kg/m^3
                gravity = 9.80665  # Acceleration due to gravity in m/s^2

                # Initialize reference depth (p0)
                if self.init_p0:
                    self.depth_p0 = (pressure_pa - surface_pressure_pa) / (water_density * gravity)
                    self.init_p0 = False
                    self.get_logger().info(f"Reference depth (p0) initialized: {self.depth_p0:.2f} m")

                # Calculate current depth
                raw_depth = (pressure_pa - surface_pressure_pa) / (water_density * gravity) - self.depth_p0
                raw_depth = max(0.0, raw_depth)  # Depth cannot be negative

                # Apply smoothing filter to depth measurements
                alpha = 0.7  # Smoothing factor between 0 and 1
                if hasattr(self, 'filtered_depth'):
                    self.filtered_depth = alpha * self.filtered_depth + (1 - alpha) * raw_depth
                else:
                    self.filtered_depth = raw_depth

                self.current_depth = self.filtered_depth

                # Publish current depth for monitoring
                depth_msg = Float64()
                depth_msg.data = self.current_depth
                self.depth_publisher.publish(depth_msg)

            if message.get_type() == 'STATUSTEXT':
                text = message.text
                self.get_logger().info(f"Status: {text}")

        except Exception as e:
            self.get_logger().error(f"Error reading MAVLink message: {e}")

    def shutdown(self):
        self.disarm()
        self.connection.close()
        self.get_logger().info("Controller shut down.")

def main(args=None):
    rclpy.init(args=args)
    node = BlueROVController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
