#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String
from bluerov2_interfaces.msg import Bar30, Attitude
from sensor_msgs.msg import BatteryState
import json


class BlueROV2CameraPublisher(Node):
    def __init__(self):
        super().__init__("bluerov2_camera_publisher")

        # Parameters
        self.declare_parameter("camera_url", "udp://192.168.2.1:5600")  # Default to BlueROV2 camera stream URL
        self.camera_url = self.get_parameter("camera_url").value

        # Publishers
        self.publisher_image = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        self.publisher_cam_info = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Subscribers
        self.battery_sub = self.create_subscription(BatteryState, "/bluerov2/battery", self.battery_callback, 10)
        self.depth_desired_sub = self.create_subscription(String, "/settings/depth/status", self.depth_desired_callback, 10)
        self.bar30_sub = self.create_subscription(Bar30, "/bluerov2/bar30", self.callback_bar30, 10)
        self.attitude_sub = self.create_subscription(Attitude, "/bluerov2/attitude", self.callback_att, 10)

        # Initialize OpenCV Video Capture
        self.cap = cv2.VideoCapture(self.camera_url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video stream: {self.camera_url}")
            raise RuntimeError("Cannot access video stream")

        # Camera Calibration Data
        self.image_width = 1280
        self.image_height = 720
        self.camera_matrix = np.array([
            [703.001903, 0.000000, 624.045812],
            [0.000000, 701.450630, 310.068328],
            [0.000000, 0.000000, 1.000000]
        ])
        self.distortion_coefficients = np.array([0.063449, -0.026851, -0.010607, 0.012980, 0.000000])
        self.rectification_matrix = np.array([
            [1.000000, 0.000000, 0.000000],
            [0.000000, 1.000000, 0.000000],
            [0.000000, 0.000000, 1.000000]
        ])
        self.projection_matrix = np.array([
            [733.242310, 0.000000, 645.635994, 0.000000],
            [0.000000, 738.102722, 299.231777, 0.000000],
            [0.000000, 0.000000, 1.000000, 0.000000]
        ])

        # Populate CameraInfo message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = self.image_width
        self.camera_info_msg.height = self.image_height
        self.camera_info_msg.k = self.camera_matrix.flatten().tolist()
        self.camera_info_msg.d = self.distortion_coefficients.tolist()
        self.camera_info_msg.r = self.rectification_matrix.flatten().tolist()
        self.camera_info_msg.p = self.projection_matrix.flatten().tolist()
        self.camera_info_msg.distortion_model = 'plumb_bob'

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create timer for publishing camera frames
        self.create_timer(0.05, self.publish_frame)  # Publish at 20 Hz

        # Debug logs
        self.get_logger().info(f"Camera publisher initialized with URL: {self.camera_url}")

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from camera")
            return

        # Resize the frame to the desired resolution
        frame = cv2.resize(frame, (self.image_width, self.image_height), interpolation=cv2.INTER_AREA)

        # Publish compressed image
        compressed_image = CompressedImage()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
        compressed_image.header.frame_id = "camera_frame"
        compressed_image.format = "jpeg"
        compressed_image.data = cv2.imencode('.jpg', frame)[1].tobytes()
        self.publisher_image.publish(compressed_image)

        # Publish camera info
        self.publisher_cam_info.publish(self.camera_info_msg)

        # Optionally display the frame
        cv2.imshow("BlueROV2 Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Stopping camera display")
            self.destroy_node()

    def battery_callback(self, msg):
        voltage = round(msg.voltage, 2)
        self.get_logger().info(f"Battery Voltage: {voltage}V")

    def depth_desired_callback(self, msg):
        data = json.loads(msg.data)
        depth_desired = abs(data.get('depth_desired', 0))
        self.get_logger().info(f"Depth Desired: {depth_desired}m")

    def callback_bar30(self, msg):
        depth = round((msg.press_abs * 100 - 103425) / (1000 * 9.81), 2)  # Convert pressure to depth
        self.get_logger().info(f"Depth: {depth}m")

    def callback_att(self, msg):
        roll = round(msg.roll, 3)
        pitch = round(msg.pitch, 3)
        yaw = round(msg.yaw, 3)
        self.get_logger().info(f"Attitude - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down BlueROV2 camera publisher")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
