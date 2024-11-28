#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Declare parameters
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_id', 0)  # Default to the first camera
        self.declare_parameter('publish_rate', 30.0)  # Default FPS rate

        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.camera_id = self.get_parameter('camera_id').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # Create a publisher
        self.publisher = self.create_publisher(CompressedImage, self.camera_topic, 10)

        # Initialize video capture
        self.capture = cv2.VideoCapture(self.camera_id)
        if not self.capture.isOpened():
            self.get_logger().error(f"Failed to open camera with ID {self.camera_id}")
            raise RuntimeError("Camera not accessible")

        self.get_logger().info(f"Publishing camera feed to topic: {self.camera_topic}")
        self.get_logger().info(f"Camera ID: {self.camera_id}, Publish Rate: {self.publish_rate} Hz")

        # Create a timer to periodically capture and publish frames
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().warning("Failed to capture frame from camera.")
            return

        # Convert the frame to JPEG format
        try:
            _, buffer = cv2.imencode('.jpg', frame)
            compressed_image = CompressedImage()
            compressed_image.header.stamp = self.get_clock().now().to_msg()
            compressed_image.header.frame_id = "camera_frame"
            compressed_image.format = "jpeg"
            compressed_image.data = buffer.tobytes()

            # Publish the frame
            self.publisher.publish(compressed_image)
            self.get_logger().info("Published a frame.")
        except Exception as e:
            self.get_logger().error(f"Failed to process and publish frame: {e}")

    def destroy_node(self):
        # Release the camera when the node is destroyed
        self.capture.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Camera Publisher.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
