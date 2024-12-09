#!/usr/bin/env python3
import rclpy
import pygame
import json
from pygame.locals import *
from rclpy.node import Node
from std_msgs.msg import UInt16, Float64, Bool, String

class Controller(Node):
    def __init__(self):
        super().__init__("input_controller_ld")  

        # Setup default parameters
        self.declare_parameter("pwm_max", 1900)
        self.declare_parameter("pwm_min", 1100)
        self.declare_parameter("pwm_neutral", 1500)
        self.declare_parameter("pwm_camera_max", 1900)
        self.declare_parameter("pwm_camera_min", 1100)
        self.declare_parameter("pwm_lights_max", 1900)
        self.declare_parameter("pwm_lights_min", 1100)
        self.declare_parameter("pwm_gripper_max", 1900)
        self.declare_parameter("pwm_gripper_min", 1100)
        self.declare_parameter("gain_pwm_cam", 400)
        self.declare_parameter("gain_pwm_lights", 50)
        self.declare_parameter("gain_depth", 0.1)
        self.declare_parameter("arm_status", False)
        self.declare_parameter("debug", True)
        self.declare_parameter("depth_min", -200)
        self.declare_parameter("depth_max", 0)

        self.pwm_min = self.get_parameter("pwm_min").value
        self.pwm_max = self.get_parameter("pwm_max").value
        self.pwm_neutral = self.get_parameter("pwm_neutral").value
        self.pwm_camera_max = self.get_parameter("pwm_camera_max").value
        self.pwm_camera_min = self.get_parameter("pwm_camera_min").value
        self.pwm_lights_max = self.get_parameter("pwm_lights_max").value
        self.pwm_lights_min = self.get_parameter("pwm_lights_min").value
        self.pwm_gripper_max = self.get_parameter("pwm_gripper_max").value
        self.pwm_gripper_min = self.get_parameter("pwm_gripper_min").value

        self.gain_depth = self.get_parameter("gain_depth").value
        self.gain_pwm_cam = self.get_parameter("gain_pwm_cam").value
        self.gain_pwm_lights = self.get_parameter("gain_pwm_lights").value
        self.gain_pwm_gripper        = self.get_parameter("gain_pwm_gripper").value
        
        self.lights_value = self.pwm_lights_min
        self.gripper_value = self.pwm_gripper_min
        self.arm = self.get_parameter("arm_status").value
        self.debug = self.get_parameter("debug").value
        self.depth_min = self.get_parameter("depth_min").value
        self.depth_max = self.get_parameter("depth_max").value

        # Node status
        self.depth_status = None

        # Publishers
        self.lights_pub = self.create_publisher(UInt16, "/bluerov2/rc/lights", 10)
        self.gripper_pub = self.create_publisher(UInt16, "/bluerov2/rc/gripper", 10)
        self.camera_tilt_pub = self.create_publisher(UInt16, "/bluerov2/rc/camera_tilt", 10)
        self.forward_pub = self.create_publisher(UInt16, "/bluerov2/rc/forward", 10)
        self.lateral_pub = self.create_publisher(UInt16, "/bluerov2/rc/lateral", 10)
        self.yaw_pub = self.create_publisher(UInt16, "/bluerov2/rc/yaw", 10)
        self.arm_pub = self.create_publisher(Bool, "/bluerov2/arm", 10)
        self.depth_controller_pub = self.create_publisher(Float64, "/settings/depth/set_depth", 10)

        # Control mode (manual or aruco-follow)
        self.control_mode = "manual"
        self.mode_pub = self.create_publisher(String, "/settings/control_mode", 10)
        self.publish_control_mode()

        # Subscriber
        self.depth_status_sub = self.create_subscription(String, "/settings/depth/status", self.callback_node_status, 10)

        # Initialize joystick
        pygame.init()
        self.joysticks = []
        for i in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.joysticks.append(joystick)
            self.get_logger().info(f"Detected joystick: {joystick.get_name()}")

        if not self.joysticks:
            self.get_logger().error("No joysticks detected!")

        # Start update loop
        self.create_timer(0.04, self.update_input)

        # Clear ROV status
        self.reset_controls()

    def publish_control_mode(self):
        mode_msg = String()
        mode_msg.data = self.control_mode
        self.mode_pub.publish(mode_msg)
        self.get_logger().info(f"Control mode: {self.control_mode}")

    def reset_controls(self):
        # Reset lights
        lights_msg = UInt16()
        lights_msg.data = self.pwm_lights_min
        self.lights_pub.publish(lights_msg)

        # Reset gripper
        gripper_msg = UInt16()
        gripper_msg.data = self.pwm_gripper_min
        self.gripper_pub.publish(gripper_msg)

        # Reset arm
        arm_msg = Bool()
        arm_msg.data = self.arm
        self.arm_pub.publish(arm_msg)

        # Reset camera
        cam_msg = UInt16()
        cam_msg.data = self.pwm_neutral
        self.camera_tilt_pub.publish(cam_msg)

    def update_input(self):
        if self.depth_status or self.debug:
            try:
                for event in pygame.event.get():
                    if event.type == JOYBUTTONDOWN:
                        self.handle_button_press(event.button)
                    elif event.type == JOYAXISMOTION:
                        self.handle_axis_motion(event.axis, event.value)
                    elif event.type == JOYHATMOTION:
                        self.handle_hat_motion(event.value)
            except Exception as e:
                self.get_logger().error(f"Joystick event handling error: {e}")
        else:
            self.get_logger().warn("Depth status unavailable. Using debug mode.")

    def handle_button_press(self, button):
        match button:
            case 4:  # Left Bumper (LB)
                self.adjust_lights("down")
            case 5:  # Right Bumper (RB)
                self.adjust_lights("up")
            case 1:  # B Button
                self.adjust_gripper("close")
            case 2:  # X Button
                self.adjust_gripper("open")
            case 7:  # Start Button
                self.arm_disarm()
            case 3:  # Y Button
                self.dive_up()
            case 0:  # A Button
                self.dive_down()
            case 6:  # Select Button
                self.toggle_control_mode()

    def handle_axis_motion(self, axis, value):
        pwm = UInt16(data=self.calculate_pwm(value))
        match axis:
            case 0: self.forward_pub.publish(pwm)
            case 1: self.lateral_pub.publish(pwm)
            case 3: self.yaw_pub.publish(pwm)

    def handle_hat_motion(self, value):
        self.camera_tilt_event(value)

    def calculate_pwm(self, value):
        if not hasattr(self, 'pwm_max') or not hasattr(self, 'pwm_neutral'):
            self.get_logger().error("PWM parameters are not initialized.")
            return self.pwm_neutral
        value = max(-1, min(1, value))
        return int(self.pwm_neutral + value * (self.pwm_max - self.pwm_neutral))

    def adjust_lights(self, direction):
        target_value = self.lights_value + self.gain_pwm_lights if direction == "up" else self.lights_value - self.gain_pwm_lights
        self.lights_value = max(self.pwm_lights_min, min(self.pwm_lights_max, target_value))
        msg = UInt16(data=self.lights_value)
        self.lights_pub.publish(msg)

    def adjust_gripper(self, direction):
        target_value = self.gripper_value + self.gain_pwm_cam if direction == "close" else self.gripper_value - self.gain_pwm_cam
        self.gripper_value = max(self.pwm_gripper_min, min(self.pwm_gripper_max, target_value))
        msg = UInt16(data=self.gripper_value)
        self.gripper_pub.publish(msg)

    def camera_tilt_event(self, value):
        msg = UInt16()
        msg.data = self.pwm_camera_max if value[1] == 1 else self.pwm_camera_min if value[1] == -1 else self.pwm_neutral
        self.camera_tilt_pub.publish(msg)

    def arm_disarm(self):
        self.arm = not self.arm
        msg = Bool(data=self.arm)
        self.arm_pub.publish(msg)
        self.get_logger().info(f"The ROV is now {'armed' if self.arm else 'disarmed'}.")

    def dive_up(self):
        new_depth = self.depth_status["depth_desired"] + self.gain_depth if self.depth_status else self.depth_max
        if new_depth <= self.depth_max:
            self.publish_depth(new_depth)

    def dive_down(self):
        new_depth = self.depth_status["depth_desired"] - self.gain_depth if self.depth_status else self.depth_min
        if new_depth >= self.depth_min:
            self.publish_depth(new_depth)

    def publish_depth(self, depth):
        msg = Float64(data=depth)
        self.depth_controller_pub.publish(msg)
        self.get_logger().info(f"Desired depth set to {depth} meters.")

    def toggle_control_mode(self):
        self.control_mode = "aruco" if self.control_mode == "manual" else "manual"
        self.publish_control_mode()

    def callback_node_status(self, msg):
        try:
            data = json.loads(msg.data)
            if data["type"] == "depth_controller":
                self.depth_status = data
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse depth status: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
