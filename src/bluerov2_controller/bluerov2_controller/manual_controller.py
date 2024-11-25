#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as mavlink
import pygame
from pygame.locals import *
from std_msgs.msg import UInt16, Bool
from sensor_msgs.msg import BatteryState
from bluerov2_interfaces.msg import Attitude, Bar30


class ManualController(Node):
    def __init__(self):
        super().__init__("manual_controller")

        # MAVLink configuration
        self.type = mavlink.MAV_TYPE_GCS
        self.autopilot = mavlink.MAV_AUTOPILOT_INVALID
        self.base_mode = mavlink.MAV_MODE_PREFLIGHT
        self.custom_mode = 0
        self.mavlink_version = 0
        self.heartbeat_period = 0.02

        # RC channel default values
        self.pitch = 1500
        self.roll = 1500
        self.throttle = 1500
        self.yaw = 1500
        self.forward = 1500
        self.lateral = 1500
        self.camera_pan = 1500
        self.camera_tilt = 1500
        self.lights = 1100
        self.gripper = 1500  # Neutral gripper position

        # ROS publishers
        self.battery_pub = self.create_publisher(BatteryState, "/bluerov2/battery", 10)
        self.arm_pub = self.create_publisher(Bool, "/bluerov2/arm_status", 10)

        # Declare parameters for MAVLink connection
        self.declare_parameter("ip", "192.168.2.1")
        self.declare_parameter("port", 14555)
        self.declare_parameter("baudrate", 115200)

        self.bluerov_ip = self.get_parameter("ip").value
        self.bluerov_port = self.get_parameter("port").value
        self.bluerov_baudrate = self.get_parameter("baudrate").value

        # Connect to BlueROV2 via MAVLink
        self.get_logger().info("Connecting to BlueROV2...")
        self.connection = mavutil.mavlink_connection(
            f"udpin:{self.bluerov_ip}:{self.bluerov_port}", baudrate=self.bluerov_baudrate
        )
        self.connection.wait_heartbeat()
        self.get_logger().info("BlueROV2 connection established.")

        self.mav = self.connection.mav
        self.target = (self.connection.target_system, self.connection.target_component)

        # Request data streams
        self.get_logger().info("Requesting data stream...")
        self.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,
            1,
        )

        # Initialize joystick
        pygame.init()
        self.joysticks = []
        for i in range(pygame.joystick.get_count()):
            self.joysticks.append(pygame.joystick.Joystick(i))
            self.joysticks[-1].init()
            self.get_logger().info(f"Joystick detected: {self.joysticks[-1].get_name()}")

        # Arm the vehicle
        self.arm()

        # Start manual control loop
        self.create_timer(0.04, self.update_inputs)

        # Start heartbeat loop
        self.create_timer(self.heartbeat_period, self.send_bluerov_commands)

    def arm(self):
        self.connection.arducopter_arm()
        self.get_logger().info("Arm requested, waiting...")
        self.connection.motors_armed_wait()
        self.get_logger().info("Thrusters armed!")

    def disarm(self):
        self.connection.arducopter_disarm()
        self.get_logger().info("Disarm requested, waiting...")
        self.connection.motors_disarmed_wait()
        self.get_logger().info("Thrusters disarmed!")

    def mapValueScalSat(self, value):
        # Scale joystick input (-1 to 1) to PWM range (1100-1900) with neutral at 1500
        pulse_width = value * 400 + 1500
        # Saturate values within range
        pulse_width = min(max(pulse_width, 1100), 1900)
        return int(pulse_width)

    def setOverrideRCIN(self, pitch, roll, throttle, yaw, forward, lateral):
        # Map thruster control values to RC channels
        rc_channel_values = (
            self.mapValueScalSat(pitch),
            self.mapValueScalSat(roll),
            self.mapValueScalSat(throttle),
            self.mapValueScalSat(yaw),
            self.mapValueScalSat(forward),
            self.mapValueScalSat(lateral),
            1500,  # Camera pan (neutral)
            1500,  # Camera tilt (neutral)
            self.lights,  # Lights control
            self.gripper,  # Gripper control
            65535, 65535, 65535, 65535, 65535, 65535  # Unused channels
        )
        self.mav.rc_channels_override_send(*self.target, *rc_channel_values)

    def send_bluerov_commands(self):
        # Example inputs (replace with actual joystick/sensor inputs)
        pitch = (self.pitch - 1500) / 400.0
        roll = (self.roll - 1500) / 400.0
        throttle = (self.throttle - 1500) / 400.0
        yaw = (self.yaw - 1500) / 400.0
        forward = (self.forward - 1500) / 400.0
        lateral = (self.lateral - 1500) / 400.0

        self.setOverrideRCIN(pitch, roll, throttle, yaw, forward, lateral)

    def update_inputs(self):
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                self.handle_joystick_motion(event)
            elif event.type == JOYBUTTONDOWN:
                self.handle_button_press(event)

    def handle_joystick_motion(self, event):
        value = event.value
        pwm_value = self.mapValueScalSat(value)

        if event.axis == 0:  # Left joystick horizontal (roll)
            self.roll = pwm_value
        elif event.axis == 1:  # Left joystick vertical (pitch)
            self.pitch = pwm_value
        elif event.axis == 2:  # Right joystick horizontal (yaw)
            self.yaw = pwm_value
        elif event.axis == 3:  # Right joystick vertical (throttle)
            self.throttle = pwm_value

    def handle_button_press(self, event):
        if event.button == 0:  # Button A: Close gripper
            self.gripper = 1900
        elif event.button == 1:  # Button B: Open gripper
            self.gripper = 1100
        elif event.button == 4:  # Left bumper: Decrease light intensity
            self.lights = max(1100, self.lights - 100)
        elif event.button == 5:  # Right bumper: Increase light intensity
            self.lights = min(1900, self.lights + 100)
        elif event.button == 7:  # Start button: Toggle arm/disarm
            self.disarm() if self.connection.motors_armed() else self.arm()

    def shutdown(self):
        self.disarm()
        self.connection.close()
        self.get_logger().info("Controller shut down.")


def main(args=None):
    rclpy.init(args=args)
    node = ManualController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()