#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tkinter import Tk, Scale, Button, Label, HORIZONTAL, Canvas, PhotoImage, Frame
from PIL import Image, ImageTk
from std_msgs.msg import UInt16, Float64, Bool, String, Float32
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class GUIController(Node):
    def __init__(self):
        super().__init__("gui_controller")

        # Setup default parameters
        self.declare_parameter("pwm_gain", 200)
        self.declare_parameter("gain_depth", 0.1)

        self.pwm_gain = self.get_parameter("pwm_gain").value
        self.gain_depth = self.get_parameter("gain_depth").value
        self.control_mode = "manual"
        self.aruco_distance = 0.0  # Initial distance

        # Publishers
        self.pwm_gain_pub = self.create_publisher(UInt16, "/settings/pwm_gain", 10)
        self.depth_controller_pub = self.create_publisher(Float64, "/settings/depth/set_depth", 10)
        self.mode_pub = self.create_publisher(String, "/settings/control_mode", 10)
        self.arm_pub = self.create_publisher(Bool, "/bluerov2/arm", 10)
        self.lights_pub = self.create_publisher(UInt16, "/bluerov2/rc/lights", 10)
        self.gripper_pub = self.create_publisher(UInt16, "/bluerov2/rc/gripper", 10)
        self.forward_pub = self.create_publisher(UInt16, "/bluerov2/rc/forward", 10)
        self.lateral_pub = self.create_publisher(UInt16, "/bluerov2/rc/lateral", 10)
        self.yaw_pub = self.create_publisher(UInt16, "/bluerov2/rc/yaw", 10)

        # Depth control variables
        self.current_depth = 0.0

        # GUI setup
        self.root = Tk()
        self.root.title("BlueROV2 GUI Controller")
        self.root.geometry("1200x800")  # Set window size

        # Frames for better layout
        self.image_frame = Frame(self.root, bg="black")
        self.image_frame.pack(fill="x", expand=True, padx=10, pady=10)
        self.control_frame = Frame(self.root, bg="#f0f0f0")
        self.control_frame.pack(fill="x", expand=True, padx=10, pady=10)
        self.button_frame = Frame(self.root, bg="#f0f0f0")
        self.button_frame.pack(fill="x", padx=10, pady=10)

        # Image display canvas
        Label(self.image_frame, text="Normal Image", bg="black", fg="white", font=("Helvetica", 12, "bold")).pack(side="left", padx=5, pady=5)
        self.normal_image_canvas = Canvas(self.image_frame, width=500, height=250, bg="black")
        self.normal_image_canvas.pack(side="left", padx=5, pady=5)

        Label(self.image_frame, text="Detected Image", bg="black", fg="white", font=("Helvetica", 12, "bold")).pack(side="right", padx=5, pady=5)
        self.detected_image_canvas = Canvas(self.image_frame, width=500, height=250, bg="black")
        self.detected_image_canvas.pack(side="right", padx=5, pady=5)

        # Subscribers for images
        self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.update_normal_image, 10)
        self.create_subscription(CompressedImage, "/aruco_detected_image", self.update_detected_image, 10)

        # Display for ArUco Distance
        Label(self.control_frame, text="ArUco Distance (m)", bg="#f0f0f0", font=("Helvetica", 12, "bold")).pack(pady=10)
        self.distance_label = Label(self.control_frame, text=f"Distance: {self.aruco_distance:.2f} m", bg="#f0f0f0", font=("Helvetica", 12), fg="blue")
        self.distance_label.pack()

        # Subscriber for ArUco distance
        self.create_subscription(Float32, "/aruco_distance", self.update_aruco_distance, 10)

        # PWM Gain Slider
        Label(self.control_frame, text="PWM Gain", bg="#f0f0f0", font=("Helvetica", 12, "bold")).pack()
        self.pwm_slider = Scale(
            self.control_frame,
            from_=100,
            to=300,
            orient=HORIZONTAL,
            command=self.update_pwm_gain,
            length=300,
            bg="lightblue",
        )
        self.pwm_slider.set(self.pwm_gain)
        self.pwm_slider.pack(pady=10)

        # Depth Slider
        Label(self.control_frame, text="Depth Control", bg="#f0f0f0", font=("Helvetica", 12, "bold")).pack()
        self.depth_slider = Scale(
            self.control_frame,
            from_=0.0,
            to=-10.0,
            resolution=0.1,
            orient=HORIZONTAL,
            command=self.update_depth,
            length=500,
            bg="lightgreen",
        )
        self.depth_slider.set(self.current_depth)
        self.depth_slider.pack(pady=10)

        # Movement and Control Buttons (2x7 Grid Layout)
        buttons = [
            ("Forward", lambda: self.move("forward"), "lightgreen"),
            ("Backward", lambda: self.move("backward"), "lightgreen"),
            ("Left", lambda: self.move("left"), "lightblue"),
            ("Right", lambda: self.move("right"), "lightblue"),
            ("Yaw Left", lambda: self.move("yaw_left"), "yellow"),
            ("Yaw Right", lambda: self.move("yaw_right"), "yellow"),
            ("Lights Up", lambda: self.adjust_lights("up"), "orange"),
            ("Lights Down", lambda: self.adjust_lights("down"), "orange"),
            ("Gripper Open", lambda: self.adjust_gripper("open"), "pink"),
            ("Gripper Close", lambda: self.adjust_gripper("close"), "pink"),
            ("Arm", self.arm, "red"),
            ("Disarm", self.disarm, "red"),
            ("Manual Mode", lambda: self.switch_mode("manual"), "purple"),
            ("ArUco Mode", lambda: self.switch_mode("aruco"), "purple"),
        ]

        for i, (text, command, color) in enumerate(buttons):
            Button(
                self.button_frame,
                text=text,
                command=command,
                bg=color,
                font=("Helvetica", 10),
                width=18,
            ).grid(row=i // 7, column=i % 7, padx=5, pady=5)

        # Main loop
        self.root.mainloop()

    def update_normal_image(self, msg):
        """Update the normal image on the canvas."""
        img = self.decode_image(msg.data)
        if img is not None:
            img_tk = ImageTk.PhotoImage(Image.fromarray(img))
            self.normal_image_canvas.create_image(0, 0, anchor="nw", image=img_tk)
            self.normal_image_canvas.image = img_tk

    def update_detected_image(self, msg):
        """Update the detected image on the canvas."""
        img = self.decode_image(msg.data)
        if img is not None:
            img_tk = ImageTk.PhotoImage(Image.fromarray(img))
            self.detected_image_canvas.create_image(0, 0, anchor="nw", image=img_tk)
            self.detected_image_canvas.image = img_tk

    def update_aruco_distance(self, msg):
        """Update the displayed ArUco distance."""
        self.aruco_distance = msg.data
        self.distance_label.config(text=f"Distance: {self.aruco_distance:.2f} m")

    def decode_image(self, img_data):
        """Decode compressed image data."""
        np_arr = np.frombuffer(img_data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is not None:
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return img

    def update_pwm_gain(self, value):
        """Update PWM gain based on slider."""
        self.pwm_gain = int(value)
        msg = UInt16()
        msg.data = self.pwm_gain
        self.pwm_gain_pub.publish(msg)
        self.get_logger().info(f"Updated PWM Gain to {self.pwm_gain}")

    def update_depth(self, value):
        """Update desired depth based on slider."""
        depth = Float64()
        depth.data = float(value)
        self.depth_controller_pub.publish(depth)
        self.get_logger().info(f"Updated Depth to {depth.data}")

    def arm(self):
        """Arm the robot."""
        msg = Bool()
        msg.data = True
        self.arm_pub.publish(msg)
        self.get_logger().info("ROV armed!")

    def disarm(self):
        """Disarm the robot."""
        msg = Bool()
        msg.data = False
        self.arm_pub.publish(msg)
        self.get_logger().info("ROV disarmed!")

    def adjust_lights(self, direction):
        """Adjust lights intensity."""
        msg = UInt16()
        if direction == "up":
            msg.data = 1900
        elif direction == "down":
            msg.data = 1100
        self.lights_pub.publish(msg)
        self.get_logger().info(f"Lights adjusted: {direction}")

    def adjust_gripper(self, direction):
        """Adjust gripper position."""
        msg = UInt16()
        if direction == "open":
            msg.data = 1900
        elif direction == "close":
            msg.data = 1100
        self.gripper_pub.publish(msg)
        self.get_logger().info(f"Gripper adjusted: {direction}")

    def move(self, direction):
        """Control forward, backward, lateral, and yaw movements."""
        msg = UInt16()
        if direction == "forward":
            msg.data = 1600
            self.forward_pub.publish(msg)
        elif direction == "backward":
            msg.data = 1400
            self.forward_pub.publish(msg)
        elif direction == "left":
            msg.data = 1400
            self.lateral_pub.publish(msg)
        elif direction == "right":
            msg.data = 1600
            self.lateral_pub.publish(msg)
        elif direction == "yaw_left":
            msg.data = 1400
            self.yaw_pub.publish(msg)
        elif direction == "yaw_right":
            msg.data = 1600
            self.yaw_pub.publish(msg)
        self.get_logger().info(f"Movement {direction} executed")

    def switch_mode(self, mode):
        """Switch between manual and ArUco modes."""
        self.control_mode = mode
        self.publish_control_mode()
        self.get_logger().info(f"Switched to {mode} mode")

    def publish_control_mode(self):
        """Publish the current control mode."""
        mode_msg = String()
        mode_msg.data = self.control_mode
        self.mode_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    gui_node = GUIController()
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
