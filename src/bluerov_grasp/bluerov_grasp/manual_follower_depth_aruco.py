#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as mavlink
from sensor_msgs.msg import PoseStamped
from bluerov2_interfaces.msg import Bar30
import pygame
from pygame.locals import *


class BlueROV2Controller(Node):
    def __init__(self):
        super().__init__('manual_follower_depth_aruco')

        # Modes
        self.manual_mode = True  # Start in manual mode
        self.depth_hold_enabled = False  # Depth hold is initially off

        # Depth hold parameters
        self.target_depth = 0.0
        self.g = 9.81
        self.p0 = 103425  # Surface pressure in Pascal
        self.rho = 1000  # Water density
        self.depth_neutral_pwm = 1500
        self.depth_max_pwm = 1900
        self.depth_min_pwm = 1100
        self.KP = 600
        self.KI = 100
        self.KD = 50
        self.depth_integral = 0.0
        self.previous_depth = 0.0
        self.previous_time = 0.0
        self.current_bar30_pressure = self.p0

        # Parameters for autonomous mode
        self.target_distance = 0.8
        self.alignment_tolerance = 0.1
        self.speed_scaling = 100
        self.default_pwm = 1500
        self.min_pwm = 1100
        self.max_pwm = 1900

        # MAVLink setup
        self.initialize_mavlink_connection()

        # Joystick setup for manual control
        self.initialize_joystick()

        # ROS Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_pose',
            self.pose_callback,
            10
        )
        self.bar30_sub = self.create_subscription(
            Bar30,
            '/bluerov2/bar30',
            self.bar30_callback,
            10
        )

        # Timer for MAVLink commands
        self.create_timer(0.04, self.send_mavlink_commands)

        self.get_logger().info("BlueROV2 Controller Initialized.")

    def initialize_mavlink_connection(self):
        # MAVLink connection parameters
        self.declare_parameter("ip", "192.168.2.1")
        self.declare_parameter("port", 14555)
        self.declare_parameter("baudrate", 115200)

        self.bluerov_ip = self.get_parameter("ip").value
        self.bluerov_port = self.get_parameter("port").value
        self.bluerov_baudrate = self.get_parameter("baudrate").value

        self.get_logger().info("Connecting to BlueROV2...")
        self.connection = mavutil.mavlink_connection(
            f"udpin:{self.bluerov_ip}:{self.bluerov_port}", baudrate=self.bluerov_baudrate
        )
        self.connection.wait_heartbeat()
        self.get_logger().info("BlueROV2 connection established.")

        # MAVLink convenience variables
        self.mav = self.connection.mav
        self.target = (self.connection.target_system, self.connection.target_component)

        # Arm the vehicle
        self.arm()

    def arm(self):
        self.connection.arducopter_arm()
        self.get_logger().info("Arming BlueROV2...")
        self.connection.motors_armed_wait()
        self.get_logger().info("Thrusters armed.")

    def disarm(self):
        self.connection.arducopter_disarm()
        self.get_logger().info("Disarming BlueROV2...")
        self.connection.motors_disarmed_wait()
        self.get_logger().info("Thrusters disarmed.")

    def initialize_joystick(self):
        pygame.init()
        self.joysticks = []
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.joysticks.append(joystick)
            self.get_logger().info(f"Joystick detected: {joystick.get_name()}")

        # Initialize RC channel values
        self.rc_channels = [
            1500,  # RC1: Gripper
            1500,  # RC2: Unused
            1500,  # RC3: Throttle
            1500,  # RC4: Camera tilt
            1500,  # RC5: Forward/Backward
            1500,  # RC6: Left/Right
            1500,  # RC7: Lights
            1500   # RC8: Yaw
        ]

    def map_to_pwm(self, value):
        pwm = int(self.default_pwm + value * self.speed_scaling)
        return min(max(pwm, self.min_pwm), self.max_pwm)

    def send_mavlink_commands(self):
        if self.manual_mode:
            self.set_manual_rc_override()
        elif self.depth_hold_enabled:
            self.set_depth_hold_rc_override()
        else:
            self.set_autonomous_rc_override()

    def set_manual_rc_override(self):
        self.update_joystick_inputs()
        self.mav.rc_channels_override_send(*self.target, *self.rc_channels)

    def update_joystick_inputs(self):
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                self.handle_joystick_motion(event)
            elif event.type == JOYBUTTONDOWN:
                self.handle_button_press(event)

    def handle_joystick_motion(self, event):
        value = event.value
        pwm_value = self.map_to_pwm(value)

        if event.axis == 0:  # Left joystick horizontal (lateral movement - RC6)
            self.rc_channels[5] = pwm_value
        elif event.axis == 1:  # Left joystick vertical (forward/backward - RC5, REVERSED)
            self.rc_channels[4] = self.map_to_pwm(-value)
        elif event.axis == 2:  # Right joystick horizontal (yaw - RC8)
            self.rc_channels[7] = pwm_value
        elif event.axis == 3:  # Left trigger (throttle - RC3)
            self.rc_channels[2] = pwm_value

    def handle_button_press(self, event):
        if event.button == 0:  # Button A: Close gripper
            self.rc_channels[0] = 1900
        elif event.button == 1:  # Button B: Open gripper
            self.rc_channels[0] = 1100
        elif event.button == 6:  # Back button: Arm/disarm
            if self.connection.motors_armed():
                self.disarm()
            else:
                self.arm()
        elif event.button == 7:  # Start button: Toggle depth hold
            self.depth_hold_enabled = not self.depth_hold_enabled
            mode = "enabled" if self.depth_hold_enabled else "disabled"
            self.get_logger().info(f"Depth hold {mode}.")
        elif event.button == 8:  # Button for mode toggle
            self.manual_mode = not self.manual_mode
            mode = "Manual" if self.manual_mode else "Autonomous"
            self.get_logger().info(f"Switched to {mode} mode.")

    def set_depth_hold_rc_override(self):
        pwm = self.calculate_depth_hold_pwm()
        self.rc_channels[2] = pwm  # RC3: Throttle for depth hold
        self.mav.rc_channels_override_send(*self.target, *self.rc_channels)

    def calculate_depth_hold_pwm(self):
        # PID control logic
        current_pressure = self.current_bar30_pressure
        depth = -(current_pressure - self.p0) / (self.rho * self.g)
        delta_depth = depth - self.previous_depth
        delta_time = self.get_clock().now().seconds_nanoseconds()[0] - self.previous_time

        # PID terms
        error = self.target_depth - depth
        self.depth_integral += error * delta_time
        derivative = delta_depth / delta_time if delta_time > 0 else 0

        # PID control output
        u = self.KP * error + self.KI * self.depth_integral - self.KD * derivative
        pwm = self.depth_neutral_pwm + u

        # Clamp PWM
        pwm = min(max(pwm, self.depth_min_pwm), self.depth_max_pwm)

        # Update previous values
        self.previous_depth = depth
        self.previous_time = self.get_clock().now().seconds_nanoseconds()[0]

        return pwm

    def set_autonomous_rc_override(self):
        self.rc_channels[4] = self.forward_pwm  # RC5: Forward/Backward
        self.rc_channels[5] = self.lateral_pwm  # RC6: Left/Right
        self.rc_channels[7] = self.yaw_pwm      # RC8: Yaw
        self.mav.rc_channels_override_send(*self.target, *self.rc_channels)

    def pose_callback(self, msg):
        if self.manual_mode or self.depth_hold_enabled:
            return

        forward_distance = msg.pose.position.z
        vertical_offset = msg.pose.position.y
        lateral_offset = msg.pose.position.x

        self.forward_pwm = self.default_pwm
        self.lateral_pwm = self.default_pwm
        self.vertical_pwm = self.default_pwm

        if abs(forward_distance - self.target_distance) > self.alignment_tolerance:
            self.forward_pwm = self.map_to_pwm(forward_distance - self.target_distance)

        if abs(lateral_offset) > self.alignment_tolerance:
            self.lateral_pwm = self.map_to_pwm(-lateral_offset)

    def bar30_callback(self, msg):
        self.current_bar30_pressure = msg.press_abs * 100  # Convert hPa to Pa

    def shutdown(self):
        self.disarm()
        self.connection.close()
        self.get_logger().info("Shutting down BlueROV2 Controller.")


def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# Back	Arm/Disarm: Toggles arm/disarm of the BlueROV2.
# Start	Depth Hold: Toggles depth hold mode on/off.
# Button 8 (Select)	Mode Toggle: Switches between Manual and Autonomous modes.