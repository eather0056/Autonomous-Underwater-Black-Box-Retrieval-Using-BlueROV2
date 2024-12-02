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
import math


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
        self.gripper = 1500  # RC1
        self.pitch = 1500    # RC2 (Unused for now)
        self.throttle = 1500  # RC3
        self.yaw = 1500  # RC4
        self.forward = 1500  # RC5
        self.lateral = 1500  # RC6
        self.lights = 1100  # RC7
        self.camera_tilt = 1500  # RC8

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
        self.create_timer(self.heartbeat_period, self.send_bluerov_commands)

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
    def map_value_scal_sat(self, value):
        # Scale joystick input (-1 to 1) to PWM range (1100-1900) with neutral at 1500
        pulse_width = value * 100 + 1500
        return int(min(max(pulse_width, 1100), 1900))

    def clamp_rc_value(self, value):
        """Clamp RC channel values to valid MAVLink range."""
        return max(1100, min(1900, int(value)))

    def send_bluerov_commands(self):
        """Send RC override commands depending on the current mode."""
        if self.mode == "manual":
            # Send manual RC commands
            self.send_rc_override(
                self.forward, self.lateral, self.throttle, self.yaw
            )

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

            if event.axis == 0:  # Left joystick horizontal (lateral movement - RC6)
                self.lateral = pwm_value
            elif event.axis == 1:  # Left joystick vertical (forward/backward - RC5, REVERSED)
                self.forward = self.map_value_scal_sat(-value)
            elif event.axis == 3:  # Right joystick horizontal (yaw - RC8)
                self.yaw = pwm_value
            elif event.axis == 4:  # Left trigger (throttle - RC3)
                self.throttle = pwm_value
            # elif event.axis == 2:  # Right trigger (camera tilt - RC4)
            #     self.camera_tilt = pwm_value

    def handle_button_press(self, event):
        
        if event.button == 7:
            if self.armed:
                self.disarm()
            else:
                self.arm()

        elif event.button == 0:  # Button A: Close gripper
            self.gripper = 1900
        elif event.button == 1:  # Button B: Open gripper
            self.gripper = 1100
        elif event.button == 4:  # Left bumper: Decrease light intensity
            self.lights = max(1100, self.lights - 100)
        elif event.button == 5:  # Right bumper: Increase light intensity
            self.lights = min(1900, self.lights + 100)
        elif event.button == 2:  # Button X: Tilt camera up
            self.camera_tilt = min(1900, self.camera_tilt + 100)  # Increase camera tilt
            #self.get_logger().info(f"Camera Tilt Increased: {self.camera_tilt}")
        elif event.button == 3:  # Button Y: Tilt camera down
            self.camera_tilt = max(1100, self.camera_tilt - 100)  # Decrease camera tilt
            #self.get_logger().info(f"Camera Tilt Decreased: {self.camera_tilt}")

        elif event.button == 6:
            self.mode = "aruco" if self.mode == "manual" else "manual"
            self.get_logger().info(f"Switched to {self.mode} mode.")

    ### ArUco Mode ###
    def pose_callback(self, msg):
        if not self.armed or self.mode != "aruco":
            return

        # Extract pose data
        forward_distance = msg.pose.position.z  # Distance from the marker
        lateral_offset = msg.pose.position.x    # Side-to-side offset
        vertical_offset = -msg.pose.position.y   # Vertical offset

        # Yaw control parameters
        yaw_kp = 150  # Proportional gain for yaw
        yaw_kd = 50   # Derivative gain for yaw
        yaw_deadband = 0.01  # Deadband for small yaw errors (in radians)

        # Calculate yaw error (using relative marker orientation to ROV)
        target_yaw = math.atan2(lateral_offset, forward_distance)  # Angle to the marker
        current_yaw = getattr(self, 'current_yaw', 0.0)  # Default to 0 if not set
        yaw_error = target_yaw - current_yaw

        # Normalize yaw error to the range [-π, π]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        # Derivative term for yaw
        delta_yaw_error = yaw_error - getattr(self, 'prev_yaw_error', 0.0)

        # Apply deadband to reduce unnecessary adjustments
        if abs(yaw_error) < yaw_deadband:
            yaw = 1500
        else:
            yaw = 1500 + int(yaw_kp * yaw_error + yaw_kd * delta_yaw_error)

        # Save current yaw and error for the next iteration
        self.prev_yaw_error = yaw_error
        self.current_yaw = target_yaw

        # Forward control parameters
        forward_kp = 80  # Proportional gain for forward
        forward_kd = 20   # Derivative gain for forward
        forward_error = forward_distance - 1.5  # Error for maintaining 1.5m distance
        delta_forward_error = forward_error - getattr(self, 'prev_forward_error', 0.0)
        forward = 1500 + int(forward_kp * forward_error + forward_kd * delta_forward_error)
        self.prev_forward_error = forward_error

        # Lateral control parameters
        lateral_kp = 150  # Proportional gain for lateral
        lateral_kd = 80   # Derivative gain for lateral
        delta_lateral_error = lateral_offset - getattr(self, 'prev_lateral_error', 0.0)
        lateral = 1500 + int(lateral_kp * lateral_offset + lateral_kd * delta_lateral_error)
        self.prev_lateral_error = lateral_offset

        # Vertical (throttle) control parameters
        throttle_kp = 180  # Proportional gain for vertical
        throttle_kd = 90   # Derivative gain for vertical
        delta_vertical_error = vertical_offset - getattr(self, 'prev_vertical_error', 0.0)
        throttle = 1500 + int(throttle_kp * vertical_offset + throttle_kd * delta_vertical_error)
        self.prev_vertical_error = vertical_offset

        # Clamp all RC values to the valid range
        forward = self.clamp_rc_value(forward)
        lateral = self.clamp_rc_value(lateral)
        throttle = self.clamp_rc_value(throttle)
        yaw = self.clamp_rc_value(yaw)

        # Send RC override commands
        self.send_rc_override(forward, lateral, throttle, yaw)

        # Log debug information
        self.get_logger().info(
            f"Aruco Mode: Forward={forward}, Lateral={lateral}, Throttle={throttle}, Yaw={yaw} "
            f"(Yaw Error={yaw_error:.2f}, Delta Yaw Error={delta_yaw_error:.2f}, Target Yaw={target_yaw:.2f}, "
            f"Forward Error={forward_error:.2f}, Lateral Offset={lateral_offset:.2f}, Vertical Offset={vertical_offset:.2f})"
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
