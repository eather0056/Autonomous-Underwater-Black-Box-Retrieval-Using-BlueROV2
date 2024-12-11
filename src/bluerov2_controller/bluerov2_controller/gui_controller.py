#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tkinter import Tk, Scale, Button, Label, HORIZONTAL, Canvas, Frame
from PIL import Image, ImageTk
from std_msgs.msg import UInt16, Float64, Bool, String, Float32
from sensor_msgs.msg import CompressedImage, Image
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
        Label(self.image_frame, text="Normal Image", bg="black", fg="white", font=("Helvetica", 12, "bold")).pack(side="top", padx=5, pady=5)
        self.normal_image_canvas = Canvas(self.image_frame, width=500, height=250, bg="black")
        self.normal_image_canvas.pack(side="left", padx=5, pady=5)

        Label(self.image_frame, text="Detected Image", bg="black", fg="white", font=("Helvetica", 12, "bold")).pack(side="top", padx=5, pady=5)
        self.detected_image_canvas = Canvas(self.image_frame, width=500, height=250, bg="black")
        self.detected_image_canvas.pack(side="right", padx=5, pady=5)

        # Subscribers for images
        self.create_subscription(Image, "/camera/image_raw/compressed", self.update_normal_image, 10)
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

        # Movement and Control Buttons
        self.create_movement_buttons()

        self.root.mainloop()

    def create_movement_buttons(self):
        buttons = [
            # Movement buttons with press/release events
            ("Forward", self.forward_pub, 1600, "lightgreen"),
            ("Backward", self.forward_pub, 1400, "lightgreen"),
            ("Left", self.lateral_pub, 1400, "lightblue"),
            ("Right", self.lateral_pub, 1600, "lightblue"),
            ("Yaw Left", self.yaw_pub, 1400, "yellow"),
            ("Yaw Right", self.yaw_pub, 1600, "yellow"),
    
            # Static action buttons
            ("Lights Up", lambda: self.adjust_lights("up"), "orange"),
            ("Lights Down", lambda: self.adjust_lights("down"), "orange"),
            ("Gripper Open", lambda: self.adjust_gripper("open"), "pink"),
            ("Gripper Close", lambda: self.adjust_gripper("close"), "pink"),
            ("Arm", self.arm, "red"),
            ("Disarm", self.disarm, "red"),
            ("Manual Mode", lambda: self.switch_mode("manual"), "purple"),
            ("ArUco Mode", lambda: self.switch_mode("aruco"), "purple"),
        ]
    
        for i, button in enumerate(buttons):
            if len(button) == 4:  # Movement buttons
                text, publisher, value, color = button
                btn = Button(
                    self.button_frame,
                    text=text,
                    bg=color,
                    font=("Helvetica", 10),
                    width=18,
                )
                btn.grid(row=i // 7, column=i % 7, padx=5, pady=5)
                btn.bind("<ButtonPress>", lambda event, pub=publisher, val=value: self.send_movement_command(pub, val))
                btn.bind("<ButtonRelease>", lambda event, pub=publisher: self.stop_movement(pub))
            elif len(button) == 3:  # Action buttons
                text, command, color = button
                Button(
                    self.button_frame,
                    text=text,
                    command=command,
                    bg=color,
                    font=("Helvetica", 10),
                    width=18,
                ).grid(row=i // 7, column=i % 7, padx=5, pady=5)
    

    def send_movement_command(self, publisher, value):
        msg = UInt16()
        msg.data = value
        publisher.publish(msg)

    def stop_movement(self, publisher):
        msg = UInt16()
        msg.data = 1500
        publisher.publish(msg)

    def update_normal_image(self, msg):
        img = self.decode_image(msg.data)
        print("image")
        if img is not None:
            img_tk = ImageTk.PhotoImage(Image.fromarray(img))
            self.normal_image_canvas.create_image(0, 0, anchor="nw", image=img_tk)
            self.normal_image_canvas.image = img_tk
            print("if")

    def update_detected_image(self, msg):
        img = self.decode_image(msg.data)
        if img is not None:
            img_tk = ImageTk.PhotoImage(Image.fromarray(img))
            self.detected_image_canvas.create_image(0, 0, anchor="nw", image=img_tk)
            self.detected_image_canvas.image = img_tk

    def update_aruco_distance(self, msg):
        self.aruco_distance = msg.data
        self.distance_label.config(text=f"Distance: {self.aruco_distance:.2f} m")

    def decode_image(self, img_data):
        np_arr = np.frombuffer(img_data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB) if img is not None else None

    def update_pwm_gain(self, value):
        self.pwm_gain = int(value)
        self.pwm_gain_pub.publish(UInt16(data=self.pwm_gain))

    def update_depth(self, value):
        self.depth_controller_pub.publish(Float64(data=float(value)))

    def arm(self):
        self.arm_pub.publish(Bool(data=True))

    def disarm(self):
        self.arm_pub.publish(Bool(data=False))

    def adjust_lights(self, direction):
        msg = UInt16(data=1900 if direction == "up" else 1100)
        self.lights_pub.publish(msg)

    def adjust_gripper(self, direction):
        msg = UInt16(data=1900 if direction == "open" else 1100)
        self.gripper_pub.publish(msg)

    def switch_mode(self, mode):
        self.mode_pub.publish(String(data=mode))


def main(args=None):
    rclpy.init(args=args)
    gui_node = GUIController()
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
