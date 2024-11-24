#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation as R  # Importing Rotation from scipy

class MarkerDetectionNode(Node):
    def __init__(self):
        super().__init__('marker_detection_node')

        # Parameters
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('marker_length', 0.07)  # Set to 0.07 meters for your 7 cm marker
        self.declare_parameter('aruco_dict_name', 'DICT_6X6_250')  # Use DICT_6X6_250 as per your marker
        self.declare_parameter('handle_offset_x', 0.0)  # Adjust as needed
        self.declare_parameter('handle_offset_y', 0.0)
        self.declare_parameter('handle_offset_z', 0.0)

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.aruco_dict_name = self.get_parameter('aruco_dict_name').get_parameter_value().string_value
        self.handle_offset = np.array([
            self.get_parameter('handle_offset_x').get_parameter_value().double_value,
            self.get_parameter('handle_offset_y').get_parameter_value().double_value,
            self.get_parameter('handle_offset_z').get_parameter_value().double_value
        ]).reshape(3, 1)

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
        self.handle_pose_pub = self.create_publisher(PoseStamped, '/handle_pose', 10)
        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

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

            # Validate camera matrix
            if not np.any(self.camera_matrix):
                self.get_logger().error("Received invalid camera matrix with all zeros.")
                return

            # Validate distortion coefficients
            if self.dist_coeffs.size == 0:
                self.get_logger().warn("Distortion coefficients are empty. Assuming zero distortion.")
                self.dist_coeffs = np.zeros((5,))

            self.get_logger().info("Camera calibration data received.")

    def image_callback(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn("Waiting for camera calibration data...")
            return

        # Convert ROS Image message to OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is not None and len(ids) > 0:
            # Estimate pose of each marker
            ret = aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )

            if ret is None:
                self.get_logger().error("Pose estimation failed.")
                return

            rvecs, tvecs, _ = ret

            for i, marker_id in enumerate(ids.flatten()):
                # Draw marker axis
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs,
                               rvecs[i], tvecs[i], self.marker_length * 0.5)

                # Calculate handle position
                handle_pos_camera = self.calculate_handle_position(rvecs[i], tvecs[i])

                # Create and publish PoseStamped message
                handle_pose = PoseStamped()
                handle_pose.header = Header()
                handle_pose.header.stamp = self.get_clock().now().to_msg()
                handle_pose.header.frame_id = 'camera_link'

                handle_pose.pose.position.x = float(handle_pos_camera[0])
                handle_pose.pose.position.y = float(handle_pos_camera[1])
                handle_pose.pose.position.z = float(handle_pos_camera[2])

                # Orientation is same as marker's orientation
                quat = self.rvec_to_quaternion(rvecs[i])
                handle_pose.pose.orientation.x = quat[0]
                handle_pose.pose.orientation.y = quat[1]
                handle_pose.pose.orientation.z = quat[2]
                handle_pose.pose.orientation.w = quat[3]

                self.handle_pose_pub.publish(handle_pose)

                # Broadcast TF from camera_link to handle
                self.broadcast_handle_tf(handle_pose, marker_id)

                # Annotate the image
                cv2.putText(frame, f"ID: {marker_id}", tuple(corners[i][0][0].astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            self.get_logger().info("No markers detected.")

        # Publish the annotated image
        try:
            annotated_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            annotated_image_msg.header.stamp = msg.header.stamp
            annotated_image_msg.header.frame_id = msg.header.frame_id
            self.annotated_image_pub.publish(annotated_image_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish annotated image: {e}")

    def calculate_handle_position(self, rvec, tvec):
        """
        Calculate the handle position in the camera frame based on the marker pose
        and the known offset.
        """
        # Convert rotation vector to rotation matrix
        R_ct, _ = cv2.Rodrigues(rvec)
        handle_pos_marker = self.handle_offset  # 3x1

        # Handle position in camera frame
        handle_pos_camera = np.dot(R_ct, handle_pos_marker) + tvec.T
        return handle_pos_camera.flatten()

    def broadcast_handle_tf(self, handle_pose, marker_id):
        """
        Broadcast a TF frame for the handle.
        """
        t = TransformStamped()
        t.header = handle_pose.header
        t.child_frame_id = f'handle_{marker_id}'

        t.transform.translation.x = handle_pose.pose.position.x
        t.transform.translation.y = handle_pose.pose.position.y
        t.transform.translation.z = handle_pose.pose.position.z

        t.transform.rotation = handle_pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def rvec_to_quaternion(self, rvec):
        """
        Convert a rotation vector to a quaternion using scipy.
        """
        # Ensure rvec is a flat array
        rvec_flat = rvec.flatten()
        # Create a Rotation object from the rotation vector
        rotation = R.from_rotvec(rvec_flat)
        # Get the quaternion (as [x, y, z, w])
        quat = rotation.as_quat()
        return quat  # Returns [x, y, z, w]

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
