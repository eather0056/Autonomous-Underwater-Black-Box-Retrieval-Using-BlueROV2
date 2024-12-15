#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import sys
import numpy as np
import cv2


class ImageViewerNode(Node):
    def __init__(self):
        super().__init__('image_viewer_node')
        self.bridge = CvBridge()
        self.image = None

        # Subscribe to the raw image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("Subscribed to /camera/image_raw")

    def image_callback(self, msg):
        """Callback to process incoming raw image data."""
        self.get_logger().info("Image received!")
        try:
            # Convert ROS2 Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convert OpenCV BGR to RGB
            self.image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


class ImageViewerApp(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node

        # Setup PyQt5 GUI
        self.setWindowTitle("ROS 2 Raw Image Viewer")
        self.layout = QVBoxLayout()
        self.label = QLabel("Waiting for image...")
        self.layout.addWidget(self.label)
        self.setLayout(self.layout)

        # Timer to refresh the GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_image)
        self.timer.start(30)  # Refresh at ~30 FPS

    def update_image(self):
        """Update the QLabel with the latest image."""
        if self.node.image is not None:
            try:
                # Convert the image to QImage format
                height, width, channel = self.node.image.shape
                bytes_per_line = channel * width
                qt_image = QImage(
                    self.node.image.data,
                    width,
                    height,
                    bytes_per_line,
                    QImage.Format_RGB888
                )
                # Update the QLabel with the new image
                self.label.setPixmap(QPixmap.fromImage(qt_image))
            except Exception as e:
                self.node.get_logger().error(f"Failed to update GUI: {e}")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # Create ROS 2 Node
    node = ImageViewerNode()

    # Create and show PyQt5 GUI
    viewer = ImageViewerApp(node)
    viewer.show()

    # Spin ROS2 node and start PyQt5 app
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
