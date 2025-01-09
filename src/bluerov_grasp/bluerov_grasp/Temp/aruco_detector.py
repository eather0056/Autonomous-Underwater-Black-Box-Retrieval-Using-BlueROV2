#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('image_topic', '/image_raw')  # Image topic to subscribe to
        self.declare_parameter('camera_info_topic', '/camera_info')  # CameraInfo topic
        self.declare_parameter('marker_length', 0.07)  # Marker size in meters (7 cm)
        self.declare_parameter('aruco_dict_name', 'DICT_6X6_250')  # Your marker's dictionary

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.aruco_dict_name = self.get_parameter('aruco_dict_name').get_parameter_value().string_value

        # Initialize ArUco dictionary
        ARUCO_DICT = {
            "DICT_4X4_50": aruco.DICT_4X4_50,
            "DICT_4X4_100": aruco.DICT_4X4_100,
            "DICT_4X4_250": aruco.DICT_4X4_250,
            "DICT_4X4_1000": aruco.DICT_4X4_1000,
            "DICT_5X5_50": aruco.DICT_5X5_50,
            "DICT_5X5_100": aruco.DICT_5X5_100,
            "DICT_5X5_250": aruco.DICT_5X5_250,
            "DICT_5X5_1000": aruco.DICT_5X5_1000,
            "DICT_6X6_50": aruco.DICT_6X6_50,
            "DICT_6X6_100": aruco.DICT_6X6_100,
            "DICT_6X6_250": aruco.DICT_6X6_250,
            "DICT_6X6_1000": aruco.DICT_6X6_1000,
            "DICT_7X7_50": aruco.DICT_7X7_50,
            "DICT_7X7_100": aruco.DICT_7X7_100,
            "DICT_7X7_250": aruco.DICT_7X7_250,
            "DICT_7X7_1000": aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL,
        }

        if self.aruco_dict_name in ARUCO_DICT:
            self.aruco_dict = aruco.Dictionary_get(ARUCO_DICT[self.aruco_dict_name])
            self.aruco_params = aruco.DetectorParameters_create()
        else:
            self.get_logger().error(f"ArUco dictionary '{self.aruco_dict_name}' not supported")
            raise ValueError(f"ArUco dictionary '{self.aruco_dict_name}' not supported")

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        # Publishers
        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image', 10)

        # CvBridge
        self.bridge = CvBridge()

        # Camera parameters
        self.camera_matrix = None
        self.dist_coeffs = None

        self.get_logger().info("Marker Detection Node Initialized")

    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            # Extract camera matrix and distortion coefficients
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

            # Print the camera matrix
            self.get_logger().info("Camera matrix received:")
            self.get_logger().info(f"\n{self.camera_matrix}")

            # Optionally, print distortion coefficients
            self.get_logger().info("Distortion coefficients received:")
            self.get_logger().info(f"{self.dist_coeffs}")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for camera calibration data...")
            return

        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Draw detected markers
            aruco.drawDetectedMarkers(frame, corners, ids)

            # Estimate pose of each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                # Draw axis for each marker
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs,
                               rvecs[i], tvecs[i], self.marker_length * 0.5)

                # Display the ID of the marker
                c = corners[i][0]
                center = c.mean(axis=0).astype(int)
                cv2.putText(frame, f"ID: {ids[i][0]}", (center[0], center[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # Print the translation and rotation vectors
                self.get_logger().info(f"Marker ID {ids[i][0]}:")
                self.get_logger().info(f"Translation Vector (tvec): {tvecs[i].flatten()}")
                self.get_logger().info(f"Rotation Vector (rvec): {rvecs[i].flatten()}")

        else:
            self.get_logger().info("No markers detected.")

        # Publish the annotated image
        try:
            annotated_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.annotated_image_pub.publish(annotated_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Marker Detection Node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
