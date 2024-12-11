#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
import cv2
import numpy as np
import random
import time


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        # Publishers
        self.raw_image_pub = self.create_publisher(CompressedImage, "/camera/image_raw/compressed", 10)
        self.detected_image_pub = self.create_publisher(CompressedImage, "/aruco_detected_image", 10)
        self.distance_pub = self.create_publisher(Float32, "/aruco_distance", 10)

        # Timer for publishing random distances
        self.create_timer(5.0, self.publish_random_distance)

        # Video capture from laptop camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open video capture device.")
            exit(1)

        self.get_logger().info("Camera Publisher started successfully.")

        # Main loop
        self.create_timer(0.1, self.publish_camera_images)

    def publish_camera_images(self):
        """Capture and publish images from the laptop camera."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image from camera.")
            return

        # Publish raw image
        raw_msg = self.encode_image_to_compressed_message(frame)
        self.raw_image_pub.publish(raw_msg)

        # Simulate marker detection by overlaying text on the frame
        detected_frame = frame.copy()
        cv2.putText(
            detected_frame, "Detected Marker", (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA
        )
        detected_msg = self.encode_image_to_compressed_message(detected_frame)
        self.detected_image_pub.publish(detected_msg)

    def publish_random_distance(self):
        """Publish a random distance value to /aruco_distance."""
        distance = random.uniform(0.5, 5.0)  # Random value between 0.5 and 5.0 meters
        distance_msg = Float32(data=distance)
        self.distance_pub.publish(distance_msg)
        self.get_logger().info(f"Published random distance: {distance:.2f} m")

    @staticmethod
    def encode_image_to_compressed_message(image):
        """Encode an image to a ROS 2 CompressedImage message."""
        msg = CompressedImage()
        msg.header.stamp = rclpy.clock.Clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
        return msg

    def destroy_node(self):
        """Ensure video capture is released when shutting down."""
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
