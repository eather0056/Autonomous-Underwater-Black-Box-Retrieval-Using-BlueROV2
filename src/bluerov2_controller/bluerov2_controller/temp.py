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
        self.camera_tilt_pub = self.create_publisher(UInt16, "/bluerov2/rc/camera_tilt", 10)

        self.yaw_kp_pub = self.create_publisher(Float64, "/settings/yaw/kp", 10)
        self.yaw_kd_pub = self.create_publisher(Float64, "/settings/yaw/kd", 10)
        self.forward_kp_pub = self.create_publisher(Float64, "/settings/forward/kp", 10)
        self.forward_kd_pub = self.create_publisher(Float64, "/settings/forward/kd", 10)
        self.lateral_kp_pub = self.create_publisher(Float64, "/settings/lateral/kp", 10)
        self.lateral_kd_pub = self.create_publisher(Float64, "/settings/lateral/kd", 10)
        self.throttle_kp_pub = self.create_publisher(Float64, "/settings/throttle/kp", 10)
        self.throttle_kd_pub = self.create_publisher(Float64, "/settings/throttle/kd", 10)
        self.forward_offset_pub = self.create_publisher(Float64, "/setings/forward_Offset_correction", 10)
        self.yaw_offset_pub = self.create_publisher(Float64, "/setings/yaw_Offset_correction", 10)
        self.lateral_offset_pub = self.create_publisher(Float64, "/setings/lateral_Offset_correction", 10)

        # Depth control variables
        self.current_depth = 0.0

        # GUI setup
        self.root = Tk()
        self.root.title("BlueROV2 GUI Controller")
        self.root.geometry("1200x800")  # Set window size

        # Configure grid weights for resizing
        self.root.grid_rowconfigure(0, weight=1)  # Image frame row
        self.root.grid_rowconfigure(1, weight=2)  # Control and button frames row
        self.root.grid_columnconfigure(0, weight=1)  # Control frame column
        self.root.grid_columnconfigure(1, weight=1)  # Button frame column

        # Frames for better layout
        self.image_frame = Frame(self.root, bg="black")
        self.image_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=10, pady=10)

        self.control_frame = Frame(self.root, bg="#f0f0f0")
        self.control_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=10)

        self.button_frame = Frame(self.root, bg="#f0f0f0")
        self.button_frame.grid(row=1, column=1, sticky="nsew", padx=10, pady=10)

        # Image display canvas
        Label(self.image_frame, text="Normal Image", bg="black", fg="white", font=("Helvetica", 12, "bold")).grid(row=0, column=0, padx=5, pady=5)
        self.normal_image_canvas = Canvas(self.image_frame, width=500, height=250, bg="black")
        self.normal_image_canvas.grid(row=1, column=0, padx=5, pady=5)

        Label(self.image_frame, text="Detected Image", bg="black", fg="white", font=("Helvetica", 12, "bold")).grid(row=0, column=1, padx=5, pady=5)
        self.detected_image_canvas = Canvas(self.image_frame, width=500, height=250, bg="black")
        self.detected_image_canvas.grid(row=1, column=1, padx=5, pady=5)

        # Subscribers for images
        self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.update_normal_image, 10)
        self.create_subscription(CompressedImage, "/aruco_detected_image", self.update_detected_image, 10)
        self.create_subscription(Float32, "/aruco_distance", self.update_aruco_distance, 10)

        # Display for ArUco Distance
        Label(self.control_frame, text="ArUco Distance (m)", bg="#f0f0f0", font=("Helvetica", 12, "bold")).grid(row=6, column=0, sticky="w", padx=5, pady=5)
        self.distance_label = Label(self.control_frame, text=f"Distance: {self.aruco_distance:.2f} m", bg="#f0f0f0", font=("Helvetica", 12), fg="blue")
        self.distance_label.grid(row=6, column=1, padx=5, pady=5)

        # Depth Control Slider
        Label(self.control_frame, text="Depth Control", bg="#f0f0f0", font=("Helvetica", 12, "bold")).grid(row=7, column=0, sticky="w", padx=5, pady=5)
        self.depth_slider = Scale(
            self.control_frame,
            from_=0.0,
            to=-10.0,
            resolution=0.1,
            orient=HORIZONTAL,
            command=self.update_depth,
            length=200,
            bg="lightgreen",
        )
        self.depth_slider.set(self.current_depth)
        self.depth_slider.grid(row=7, column=1, padx=5, pady=5)

        # PWM Gain Slider
        Label(self.control_frame, text="PWM Gain", bg="#f0f0f0", font=("Helvetica", 12, "bold")).grid(row=8, column=0, sticky="w", padx=5, pady=5)
        self.pwm_slider = Scale(
            self.control_frame,
            from_=100,
            to=300,
            orient=HORIZONTAL,
            command=self.update_pwm_gain,
            length=200,
            bg="lightblue",
        )
        self.pwm_slider.set(self.pwm_gain)
        self.pwm_slider.grid(row=8, column=1, padx=5, pady=5)

        # Add sliders to the control frame
        self.add_slider("Yaw KP", self.control_frame, 0.0, 100.0, 40, self.yaw_kp_pub, 0, 0)
        self.add_slider("Yaw KD", self.control_frame, 0.0, 50.0, 20, self.yaw_kd_pub, 0, 2)
        self.add_slider("Forward KP", self.control_frame, 0.0, 100.0, 79, self.forward_kp_pub, 1, 0)
        self.add_slider("Forward KD", self.control_frame, 0.0, 50.0, 30, self.forward_kd_pub, 1, 2)
        self.add_slider("Lateral KP", self.control_frame, 0.0, 50.0, 30, self.lateral_kp_pub, 2, 0)
        self.add_slider("Lateral KD", self.control_frame, 0.0, 50.0, 5, self.lateral_kd_pub, 2, 2)
        self.add_slider("Throttle KP", self.control_frame, 0.0, 300.0, 200, self.throttle_kp_pub, 3, 0)
        self.add_slider("Throttle KD", self.control_frame, 0.0, 100.0, 30, self.throttle_kd_pub, 3, 2)
        self.add_slider("Forward Offset", self.control_frame, 0.0, 2.0, 0.7, self.forward_offset_pub, 4, 0)
        self.add_slider("Yaw Offset", self.control_frame, -20.0, 20.0, -10, self.yaw_offset_pub, 4, 2)
        self.add_slider("Lateral Offset", self.control_frame, 0.0, 2.0, 0.4, self.lateral_offset_pub, 5, 0)

        # Movement and Control Buttons
        self.create_movement_buttons()

        self.root.mainloop()

    def create_movement_buttons(self):
        # Movement buttons with press/release events
        movement_buttons = [
            ("Forward", self.forward_pub, 1600, "lightgreen"),
            ("Backward", self.forward_pub, 1400, "lightgreen"),
            ("Left", self.lateral_pub, 1400, "lightblue"),
            ("Right", self.lateral_pub, 1600, "lightblue"),
            ("Yaw Left", self.yaw_pub, 1400, "yellow"),
            ("Yaw Right", self.yaw_pub, 1600, "yellow"),
        ]

        # Static action buttons
        action_buttons = [
            ("Lights Up", lambda: self.adjust_lights("up"), "orange"),
            ("Lights Down", lambda: self.adjust_lights("down"), "orange"),
            ("Gripper Open", lambda: self.adjust_gripper("open"), "pink"),
            ("Gripper Close", lambda: self.adjust_gripper("close"), "pink"),
            ("Arm", self.arm, "red"),
            ("Disarm", self.disarm, "red"),
            ("Manual Mode", lambda: self.switch_mode("manual"), "purple"),
            ("ArUco Mode", lambda: self.switch_mode("aruco"), "purple"),
        ]

        # Place movement buttons in rows of 4
        for i, button in enumerate(movement_buttons):
            text, publisher, value, color = button
            btn = Button(
                self.button_frame,
                text=text,
                bg=color,
                font=("Helvetica", 10),
                width=15,
            )
            btn.grid(row=i // 6, column=i % 6, padx=5, pady=5)  # Arrange in rows of 4
            btn.bind("<ButtonPress>", lambda event, pub=publisher, val=value: self.send_movement_command(pub, val))
            btn.bind("<ButtonRelease>", lambda event, pub=publisher: self.stop_movement(pub))

        # Place action buttons in rows of 4 (starting below movement buttons)
        for i, button in enumerate(action_buttons):
            text, command, color = button
            Button(
                self.button_frame,
                text=text,
                command=command,
                bg=color,
                font=("Helvetica", 10),
                width=15,
            ).grid(row=(len(movement_buttons) // 6) + (i // 6), column=i % 6, padx=5, pady=5)

    def add_slider(self, label, frame, from_, to, default, publisher, row, column):
        """Add a slider to the specified row and column."""
        Label(frame, text=label, bg="#f0f0f0", font=("Helvetica", 12, "bold")).grid(row=row, column=column, sticky="w", padx=5, pady=5)
        slider = Scale(
            frame,
            from_=from_,
            to=to,
            resolution=0.1,
            orient=HORIZONTAL,
            command=lambda value: self.update_param(value, publisher),
            length=200,  # Adjust the length to fit in the grid
            bg="lightblue",
        )
        slider.set(default)
        slider.grid(row=row, column=column + 1, padx=5, pady=5)

    def update_param(self, value, publisher):
        """Publish the updated value to the specified topic."""
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)
        self.get_logger().info(f"Updated {publisher.topic_name} to {value}")

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
        if img is not None:
            img_tk = ImageTk.PhotoImage(Image.fromarray(img))
            self.normal_image_canvas.create_image(0, 0, anchor="nw", image=img_tk)
            self.normal_image_canvas.image = img_tk

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