#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tkinter import Tk, Scale, Button, Label, HORIZONTAL, Canvas, Frame
from PIL import Image, ImageTk
from std_msgs.msg import UInt16, Float64, Bool, String, Float32
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np

class CameraGUI(Node):
    def __init__(self):
        super().__init__('camera_gui')

        # ROS 2 Subscribers
        self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.raw_image_callback, 10)
        self.create_subscription(CompressedImage, '/aruco_detected_image', self.detected_image_callback, 10)
        self.create_subscription(Float32, '/aruco_distance', self.distance_callback, 10)

        self.raw_image = None
        self.detected_image = None
        self.distance = "N/A"

        # Tkinter GUI Setup
        self.root = Tk()
        self.root.title("Camera GUI")

        self.canvas_raw = Canvas(self.root, width=640, height=480)
        self.canvas_raw.pack()

        self.canvas_detected = Canvas(self.root, width=640, height=480)
        self.canvas_detected.pack()

        self.distance_label = Label(self.root, text="Distance: N/A", font=("Arial", 16))
        self.distance_label.pack()

        self.update_gui()

    def raw_image_callback(self, msg):
        self.get_logger().info("Raw image received")
        self.raw_image = self.decode_compressed_image(msg)

    def detected_image_callback(self, msg):
        self.get_logger().info("Detected image received")
        self.detected_image = self.decode_compressed_image(msg)

    def distance_callback(self, msg):
        self.get_logger().info(f"Distance received: {msg.data:.2f} m")
        self.distance = f"{msg.data:.2f} m"

    def update_gui(self):
        self.get_logger().info("Updating GUI")
        if self.raw_image is not None:
            self.display_image(self.canvas_raw, self.raw_image)

        if self.detected_image is not None:
            self.display_image(self.canvas_detected, self.detected_image)

        self.distance_label.config(text=f"Distance: {self.distance}")

        self.root.after(100, self.update_gui)

    def decode_compressed_image(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Failed to decode image: {e}")
            return None

    def display_image(self, canvas, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image)
        image_tk = ImageTk.PhotoImage(image=image)
        canvas.delete("all")  # Clear previous image
        canvas.create_image(0, 0, anchor="nw", image=image_tk)
        canvas.image = image_tk


    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = CameraGUI()

    try:
        rclpy.spin(node, executor=None)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

    node.run()

if __name__ == "__main__":
    main()