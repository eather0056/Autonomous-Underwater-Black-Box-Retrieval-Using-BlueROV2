#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class MarkerDetectionSimulationNode(Node):
    def __init__(self):
        super().__init__('marker_detection_simulation')

        self.get_logger().info("Initializing Marker Detection Simulation Node...")

        # Parameters
        self.declare_parameter('image_topic', '/camera/image/compressed')
        self.declare_parameter('camera_info_file', 'camera_info.yaml')
        self.declare_parameter('marker_length', 0.1)
        self.declare_parameter('aruco_dict_name', 'DICT_4X4_250')
        self.declare_parameter('handle_offset_x', 0.0)
        self.declare_parameter('handle_offset_y', 0.0)
        self.declare_parameter('handle_offset_z', 0.0)

        self.get_logger().info("Loading parameters...")
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_file = self.get_parameter('camera_info_file').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.aruco_dict_name = self.get_parameter('aruco_dict_name').get_parameter_value().string_value
        self.handle_offset = np.array([
            self.get_parameter('handle_offset_x').get_parameter_value().double_value,
            self.get_parameter('handle_offset_y').get_parameter_value().double_value,
            self.get_parameter('handle_offset_z').get_parameter_value().double_value
        ]).reshape(3, 1)

        self.get_logger().info("Parameters loaded successfully.")

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

        self.get_logger().info(f"Selected ArUco dictionary: {self.aruco_dict_name}")
        if self.aruco_dict_name in ARUCO_DICT:
            self.aruco_dict = aruco.Dictionary_get(ARUCO_DICT[self.aruco_dict_name])
            self.aruco_params = aruco.DetectorParameters_create()
        else:
            self.get_logger().error(f"ArUco dictionary '{self.aruco_dict_name}' not supported.")
            raise ValueError(f"ArUco dictionary '{self.aruco_dict_name}' not supported.")

        # Subscribers
        self.get_logger().info("Setting up subscribers and publishers...")
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.handle_pose_pub = self.create_publisher(PoseStamped, '/handle_pose', 10)
        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # CvBridge
        self.bridge = CvBridge()

        # Load camera calibration data
        self.get_logger().info("Loading camera calibration data...")
        self.load_camera_info(self.camera_info_file)
        self.get_logger().info("Marker Detection Simulation Node Initialized.")

    def load_camera_info(self, file_name):
        self.get_logger().debug("Locating camera info file...")
        package_share_directory = get_package_share_directory('bluerov_grasp')
        config_directory = os.path.join(package_share_directory, 'config')
        file_path = os.path.join(config_directory, file_name)

        self.get_logger().debug(f"Checking existence of file: {file_path}")
        if not os.path.exists(file_path):
            self.get_logger().error(f"Camera info file '{file_path}' not found.")
            raise FileNotFoundError(f"Camera info file '{file_path}' not found.")

        with open(file_path, 'r') as file:
            camera_info = yaml.safe_load(file)

        self.camera_matrix = np.array(camera_info['camera_matrix']['data']).reshape((3, 3))
        self.dist_coeffs = np.array(camera_info['distortion_coefficients']['data'])
        self.get_logger().debug(f"Camera matrix: {self.camera_matrix}")
        self.get_logger().debug(f"Distortion coefficients: {self.dist_coeffs}")

    def image_callback(self, msg):
        self.get_logger().info("Received an image.")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().debug("Converted ROS image to OpenCV image.")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.get_logger().debug("Converted image to grayscale.")

        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        self.get_logger().info(f"Detected markers: {ids if ids is not None else 'None'}")

        if ids is not None:
            self.get_logger().info(f"Processing {len(ids)} markers.")
            ret = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )
            if ret is None:
                self.get_logger().error("Pose estimation failed.")
                return
            rvecs, tvecs, _ = ret
            for i, marker_id in enumerate(ids.flatten()):
                self.get_logger().debug(f"Processing marker ID: {marker_id}")
                handle_pos_camera = self.calculate_handle_position(rvecs[i], tvecs[i])
                self.get_logger().debug(f"Handle position in camera frame: {handle_pos_camera}")
        else:
            self.get_logger().info("No markers detected.")

        self.get_logger().debug("Publishing annotated image.")

    def calculate_handle_position(self, rvec, tvec):
        self.get_logger().debug("Calculating handle position...")
        R_ct, _ = cv2.Rodrigues(rvec)
        handle_pos_camera = np.dot(R_ct, self.handle_offset) + tvec.T
        return handle_pos_camera.flatten()

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectionSimulationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Marker Detection Simulation Node shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
