#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import pygame
from pygame.locals import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Bool


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

        # Depth hold parameters
        self.target_depth = 0.0
        self.depth_neutral_pwm = 1500
        self.depth_max_pwm = 1900
        self.depth_min_pwm = 1100
        self.KP = 600
        self.KI = 100
        self.KD = 50
        self.depth_integral = 0.0
        self.previous_depth = 0.0
        self.previous_time = self.get_clock().now()
        self.current_depth = 0.0

        # Autonomous mode gains (lowered for smoother movements)
        self.forward_gain = 50
        self.lateral_gain = 50
        self.vertical_gain = 50

        # Subscribe to ArUco pose topic and pressure data
        self.pose_sub = self.create_subscription(
            PoseStamped, '/aruco_pose', self.pose_callback, 10)
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/bluerov2/scaled_pressure2', self.pressure_callback, 10)

        # Start update loops
        self.start_heartbeat_and_input_loops()
        self.get_logger().info("BlueROV Controller Initialized.")

    ### Initialization Functions ###
    def initialize_mavlink_config(self):
        self.heartbeat_period = 0.02  # Heartbeat every 20 ms

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
        self.create_timer(0.04, self.update_inputs)  # Joystick input loop
        self.create_timer(self.heartbeat_period, self.send_bluerov_commands)  # Sends commands to BlueROV

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
        return self.clamp_rc_value(value * 100 + 1500)

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
            self.send_depth_hold_commands()

    ### Input Handling ###
    def update_inputs(self):
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                self.handle_joystick_motion(event)
            elif event.type == JOYBUTTONDOWN:
                self.handle_button_press(event)

    def handle_joystick_motion(self, event):
        if self.mode == "manual":
            value = event.value
            pwm_value = self.map_value_scal_sat(value)

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
    def pressure_callback(self, msg):
        # Convert pressure (in Pa) to depth (in meters)
        surface_pressure_pa = 103425.0
        water_density = 1000.0
        gravity = 9.81
        self.current_depth = -(msg.fluid_pressure - surface_pressure_pa) / (water_density * gravity)

    def calculate_depth_pwm(self):
        # Correct depth error calculation for PID control
        error = self.target_depth + self.current_depth  # Invert error to ensure correct direction
        current_time = self.get_clock().now()
        delta_time = (current_time - self.previous_time).nanoseconds * 1e-9
        self.previous_time = current_time

        # PID terms
        self.depth_integral += error * delta_time
        depth_derivative = (self.current_depth - self.previous_depth) / delta_time
        self.previous_depth = self.current_depth

        # PID control output
        pid_output = (self.KP * error) + (self.KI * self.depth_integral) - (self.KD * depth_derivative)

        # Convert PID output to throttle PWM
        throttle_pwm = self.clamp_rc_value(self.depth_neutral_pwm + pid_output)
        return throttle_pwm



    def send_depth_hold_commands(self):
        pwm = self.calculate_depth_pwm()
        self.send_rc_override(self.forward, self.lateral, pwm, self.yaw)

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
