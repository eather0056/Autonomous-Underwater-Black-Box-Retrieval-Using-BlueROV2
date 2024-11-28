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
        self.initialize_mavlink_config()
        self.initialize_rc_defaults()
        self.initialize_ros_publishers()
        self.initialize_mavlink_connection()
        self.initialize_joystick()
        self.start_heartbeat_and_input_loops()

    ### Initialization Functions ###
    def initialize_mavlink_config(self):
        # MAVLink configuration
        self.type = mavlink.MAV_TYPE_GCS
        self.autopilot = mavlink.MAV_AUTOPILOT_INVALID
        self.base_mode = mavlink.MAV_MODE_PREFLIGHT
        self.custom_mode = 0
        self.mavlink_version = 0
        self.heartbeat_period = 0.02  # Heartbeat every 20 ms

    def initialize_rc_defaults(self):
        # RC channel default values
        self.gripper = 1500  # RC1
        self.pitch = 1500    # RC2 (Unused for now)
        self.throttle = 1500  # RC3
        self.camera_tilt = 1500  # RC4 (Interchanged with yaw)
        self.forward = 1500  # RC5
        self.lateral = 1500  # RC6
        self.lights = 1500  # RC7
        self.yaw = 1500  # RC8

    def initialize_ros_publishers(self):
        # ROS publishers
        self.battery_pub = self.create_publisher(BatteryState, "/bluerov2/battery", 10)
        self.arm_pub = self.create_publisher(Bool, "/bluerov2/arm_status", 10)

    def initialize_mavlink_connection(self):
        # Setup connection parameters
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

        # MAVLink convenience variables
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

        # Arm the vehicle
        self.arm()

    def initialize_joystick(self):
        # Initialize joystick
        pygame.init()
        self.joysticks = []
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.joysticks.append(joystick)
            self.get_logger().info(f"Joystick detected: {joystick.get_name()}")

    def start_heartbeat_and_input_loops(self):
        # Start input and heartbeat loops
        self.create_timer(0.04, self.update_inputs)
        self.create_timer(self.heartbeat_period, self.send_bluerov_commands)

    ### MAVLink Command Functions ###
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

    def map_value_scal_sat(self, value):
        # Scale joystick input (-1 to 1) to PWM range (1100-1900) with neutral at 1500
        pulse_width = value * 100 + 1500
        return int(min(max(pulse_width, 1100), 1900))

    def set_override_rcin(self):
        # Map RC channel values
        rc_channel_values = (
            self.gripper,      # RC1: Gripper
            1500,              # RC2: (Unused for now)
            self.throttle,     # RC3: Throttle
            self.camera_tilt,  # RC4: Camera tilt
            self.forward,      # RC5: Forward/Backward
            self.lateral,      # RC6: Left/Right
            self.lights,       # RC7: Lights
            self.yaw,          # RC8: Yaw
            65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535  # Unused channels
        )
        self.mav.rc_channels_override_send(*self.target, *rc_channel_values)

    def send_bluerov_commands(self):
        self.set_override_rcin()

    ### Input Handling Functions ###
    def update_inputs(self):
        for event in pygame.event.get():
            if event.type == JOYAXISMOTION:
                self.handle_joystick_motion(event)
            elif event.type == JOYBUTTONDOWN:
                self.handle_button_press(event)

    def handle_joystick_motion(self, event):
        value = event.value
        pwm_value = self.map_value_scal_sat(value)

        if event.axis == 0:  # Left joystick horizontal (lateral movement - RC6)
            self.lateral = pwm_value
        elif event.axis == 1:  # Left joystick vertical (forward/backward - RC5, REVERSED)
            self.forward = self.map_value_scal_sat(-value)
        elif event.axis == 2:  # Right joystick horizontal (yaw - RC8)
            self.yaw = pwm_value
        elif event.axis == 3:  # Left trigger (throttle - RC3)
            self.throttle = pwm_value
        elif event.axis == 4:  # Right trigger (camera tilt - RC4)
            self.camera_tilt = pwm_value

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
            if self.connection.motors_armed():
                self.disarm()
            else:
                self.arm()

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
