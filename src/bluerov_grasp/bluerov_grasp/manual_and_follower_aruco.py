#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as mavlink
from sensor_msgs.msg import PoseStamped
import pygame
from pygame.locals import *


class BlueROV2Controller(Node):
    def __init__(self):
        super().__init__('manual_and_follower_aruco')

        # Modes
        self.manual_mode = True  # Start in manual mode

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

        # Initialize joystick values for manual control
        self.rc_channels = [1500] * 8

    def map_to_pwm(self, value):
        pwm = int(self.default_pwm + value * self.speed_scaling)
        return min(max(pwm, self.min_pwm), self.max_pwm)

    def send_mavlink_commands(self):
        if self.manual_mode:
            self.set_manual_rc_override()
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

        if event.axis == 0:  # Left joystick horizontal (lateral movement)
            self.rc_channels[5] = pwm_value
        elif event.axis == 1:  # Left joystick vertical (forward/backward, REVERSED)
            self.rc_channels[4] = self.map_to_pwm(-value)
        elif event.axis == 2:  # Right joystick horizontal (yaw)
            self.rc_channels[7] = pwm_value
        elif event.axis == 3:  # Left trigger (throttle)
            self.rc_channels[2] = pwm_value

    def handle_button_press(self, event):
        if event.button == 0:  # Button A: Close gripper
            self.rc_channels[0] = 1900
        elif event.button == 1:  # Button B: Open gripper
            self.rc_channels[0] = 1100
        elif event.button == 7:  # Start button: Toggle arm/disarm or switch mode
            if self.manual_mode:
                self.manual_mode = False
                self.get_logger().info("Switched to Autonomous Mode.")
            else:
                self.manual_mode = True
                self.get_logger().info("Switched to Manual Mode.")
        elif event.button == 6:  # Back button: Arm/disarm
            if self.connection.motors_armed():
                self.disarm()
            else:
                self.arm()

    def set_autonomous_rc_override(self):
        rc_channels = [1500] * 8
        rc_channels[2] = self.vertical_pwm
        rc_channels[4] = self.forward_pwm
        rc_channels[5] = self.lateral_pwm
        self.mav.rc_channels_override_send(*self.target, *rc_channels)

    def pose_callback(self, msg):
        if self.manual_mode:
            return  # Ignore pose updates in manual mode

        forward_distance = msg.pose.position.z
        vertical_offset = msg.pose.position.y
        lateral_offset = msg.pose.position.x

        # Compute RC commands
        self.forward_pwm = self.default_pwm
        self.lateral_pwm = self.default_pwm
        self.vertical_pwm = self.default_pwm

        # Forward/backward motion
        if abs(forward_distance - self.target_distance) > self.alignment_tolerance:
            self.forward_pwm = self.map_to_pwm(forward_distance - self.target_distance)

        # Lateral alignment (side-to-side)
        if abs(lateral_offset) > self.alignment_tolerance:
            self.lateral_pwm = self.map_to_pwm(-lateral_offset)

        # Vertical alignment (up/down)
        if abs(vertical_offset) > self.alignment_tolerance:
            self.vertical_pwm = self.map_to_pwm(-vertical_offset)

        self.get_logger().info(
            f"Forward PWM: {self.forward_pwm}, Lateral PWM: {self.lateral_pwm}, Vertical PWM: {self.vertical_pwm}"
        )

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
