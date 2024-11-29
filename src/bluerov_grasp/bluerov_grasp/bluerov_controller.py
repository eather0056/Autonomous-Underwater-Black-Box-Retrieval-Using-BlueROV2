#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
import pymavlink.dialects.v20.ardupilotmega as mavlink
import pygame
from pygame.locals import *
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16, Bool
from sensor_msgs.msg import BatteryState
from bluerov2_interfaces.msg import Attitude, Bar30


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
        self.mode = "manual"  # Modes: "manual" or "aruco"
        self.depth_hold_active = False  # Track depth hold mode status

        # Subscribe to ArUco pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_pose',
            self.pose_callback,
            10
        )

        # Start update loops
        self.start_heartbeat_and_input_loops()
        self.get_logger().info("BlueROV Controller Initialized.")

    ### Initialization Functions ###
    def initialize_mavlink_config(self):
        self.heartbeat_period = 0.02  # Heartbeat every 20 ms

    def initialize_rc_defaults(self):
        # RC channel default values
        self.gripper = 1500    # RC1
        self.pitch = 1500      # RC2 (Unused for now)
        self.throttle = 1500   # RC3
        self.yaw = 1500        # RC4
        self.forward = 1500    # RC5
        self.lateral = 1500    # RC6
        self.lights = 1500     # RC7
        self.camera_tilt = 1500# RC8

    def initialize_ros_publishers(self):
        self.battery_pub = self.create_publisher(BatteryState, "/bluerov2/battery", 10)
        self.arm_pub = self.create_publisher(Bool, "/bluerov2/arm_status", 10)

    def initialize_mavlink_connection(self):
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

        self.mav = self.connection.mav
        self.target = (self.connection.target_system, self.connection.target_component)

        self.mav.request_data_stream_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            4,
            1,
        )

    def initialize_joystick(self):
        pygame.init()
        self.joysticks = []
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.joysticks.append(joystick)
            self.get_logger().info(f"Joystick detected: {joystick.get_name()}")

    def start_heartbeat_and_input_loops(self):
        self.create_timer(0.04, self.update_inputs)
        self.create_timer(self.heartbeat_period, self.send_heartbeat)
        self.create_timer(1.0, self.check_mode)  # Check ROV mode periodically

    ### MAVLink Command Functions ###
    def arm(self):
        if not self.armed:
            self.connection.arducopter_arm()
            self.get_logger().info("Arm requested, waiting...")
            self.connection.motors_armed_wait()
            self.armed = True
            self.get_logger().info("Thrusters armed!")

    def disarm(self):
        if self.armed:
            self.connection.arducopter_disarm()
            self.get_logger().info("Disarm requested, waiting...")
            self.connection.motors_disarmed_wait()
            self.armed = False
            self.get_logger().info("Thrusters disarmed!")

    def clamp_rc_value(self, value):
        """Clamp RC channel values to valid MAVLink range."""
        return max(1100, min(1900, int(value)))

    def send_heartbeat(self):
        """Send heartbeat to maintain connection."""
        self.mav.heartbeat_send(
            mavlink.MAV_TYPE_GCS,
            mavlink.MAV_AUTOPILOT_INVALID,
            mavlink.MAV_MODE_PREFLIGHT,
            0,
            mavlink.MAV_STATE_ACTIVE
        )

    def check_mode(self):
        """Check the current mode of the ROV."""
        try:
            msg = self.connection.recv_match(type='HEARTBEAT', blocking=False)
            if msg:
                mode = self.decode_mode(msg.custom_mode)
                self.get_logger().info(f"Current Mode: {mode}")
                if mode == "Depth Hold":
                    self.depth_hold_active = True
                else:
                    self.depth_hold_active = False
        except Exception as e:
            self.get_logger().warn(f"Failed to check mode: {e}")

    def decode_mode(self, custom_mode):
        """Decode custom_mode to a human-readable string."""
        modes = {
            0: "Stabilize",
            1: "Acro",
            2: "Altitude Hold",
            3: "Auto",
            4: "Guided",
            5: "Loiter",
            6: "RTL",
            7: "Circle",
            9: "Land",
            16: "Position",
            19: "Depth Hold",
        }
        return modes.get(custom_mode, "Unknown")

    def send_bluerov_commands(self):
        """Send RC override commands depending on the current mode."""
        if self.mode == "manual":
            # Send manual RC commands
            if self.depth_hold_active:
                # Do not override RC3 (throttle) in Depth Hold mode
                self.send_rc_override(self.forward, self.lateral, 1500, self.yaw)
            else:
                self.send_rc_override(self.forward, self.lateral, self.throttle, self.yaw)

    def send_rc_override(self, forward, lateral, throttle, yaw):
        """Send RC override commands to BlueROV."""
        rc_channel_values = (
            self.clamp_rc_value(self.gripper),    # RC1
            1500,                                 # RC2 (Unused for now)
            self.clamp_rc_value(throttle),        # RC3
            self.clamp_rc_value(yaw),             # RC4
            self.clamp_rc_value(forward),         # RC5
            self.clamp_rc_value(lateral),         # RC6
            self.clamp_rc_value(self.lights),     # RC7
            self.clamp_rc_value(self.camera_tilt),# RC8
            65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535  # Unused channels
        )
        self.mav.rc_channels_override_send(*self.target, *rc_channel_values)

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
            pwm_value = self.clamp_rc_value(value * 100 + 1500)

            if event.axis == 0:
                self.lateral = pwm_value
            elif event.axis == 1:
                self.forward = self.clamp_rc_value(-value * 100 + 1500)
            elif event.axis == 3:
                self.yaw = pwm_value
            elif event.axis == 4:
                self.throttle = pwm_value
            elif event.axis == 2:  # Right trigger (camera tilt - RC4)
                self.camera_tilt = pwm_value

    def handle_button_press(self, event):
        if event.button == 7:
            if self.armed:
                self.disarm()
            else:
                self.arm()
        elif event.button == 6:
            self.mode = "aruco" if self.mode == "manual" else "manual"
            self.get_logger().info(f"Switched to {self.mode} mode.")
        elif event.button == 0:  # Button A: Close gripper
            self.gripper = 1900
        elif event.button == 1:  # Button B: Open gripper
            self.gripper = 1100
        elif event.button == 4:  # Left bumper: Decrease light intensity
            self.lights = max(1100, self.lights - 100)
        elif event.button == 5:  # Right bumper: Increase light intensity
            self.lights = min(1900, self.lights + 100)

    ### ArUco Mode ###
    def pose_callback(self, msg):
        if not self.armed or self.mode != "aruco":
            return

        forward_distance = msg.pose.position.z
        lateral_offset = msg.pose.position.x
        vertical_offset = msg.pose.position.y

        forward = 1500 + int((1.5 - forward_distance) * 200)
        lateral = 1500 + int(lateral_offset * 100)
        throttle = 1500 + int(vertical_offset * 100)

        self.send_rc_override(forward, lateral, throttle, self.yaw)

        self.get_logger().info(
            f"Aruco Mode: Forward={forward}, Lateral={lateral}, Throttle={throttle}"
        )

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
