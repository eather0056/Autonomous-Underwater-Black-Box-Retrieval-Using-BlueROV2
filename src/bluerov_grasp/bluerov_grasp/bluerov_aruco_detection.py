#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header


class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')

        # Parameters
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('marker_length', 0.12)  # Marker size in meters
        self.declare_parameter('aruco_dict_name', 'DICT_5X5_250')  # ArUco dictionary
        self.declare_parameter('camera_matrix', [715.919995, 0.000000, 558.212919, 0.000000, 718.924099, 289.680513, 0.0, 0.0, 1.000000])  # Default flat camera matrix
        self.declare_parameter('dist_coeffs', [0.055901, -0.016813, -0.015709, -0.014352, 0.000000])  # Default distortion coefficients

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.aruco_dict_name = self.get_parameter('aruco_dict_name').get_parameter_value().string_value
        camera_matrix_flat = self.get_parameter('camera_matrix').get_parameter_value().double_array_value
        self.camera_matrix = np.array(camera_matrix_flat).reshape(3, 3)  # Convert to 3x3 matrix
        self.dist_coeffs = np.array(self.get_parameter('dist_coeffs').get_parameter_value().double_array_value)

        self.get_logger().info(f"Subscribed to image topic: {self.image_topic}")
        self.get_logger().info(f"ArUco dictionary: {self.aruco_dict_name}")
        self.get_logger().info(f"Marker length: {self.marker_length} meters")
        self.get_logger().info(f"Camera matrix: {self.camera_matrix}")
        self.get_logger().info(f"Distortion coefficients: {self.dist_coeffs}")

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
            self.get_logger().error(f"ArUco dictionary '{self.aruco_dict_name}' not supported.")
            raise ValueError(f"ArUco dictionary '{self.aruco_dict_name}' not supported.")

        # Subscribers
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10
        )

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.image_pub = self.create_publisher(CompressedImage, '/aruco_detected_image', 10)
        self.distance_pub = self.create_publisher(Float32, '/aruco_distance', 10)

        # CvBridge
        self.bridge = CvBridge()

        self.get_logger().info("ArUco Detection Node Initialized.")

    def image_callback(self, msg):
        self.get_logger().info("Received an image.")

        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.get_logger().info("Image decompressed successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to decompress image: {e}")
            return

        # Display the raw image
        try:
            if frame is not None:
                cv2.imshow("Raw Image", frame)
                cv2.waitKey(1)  # This allows OpenCV to update the display
                self.get_logger().info("Displaying raw image.")
            else:
                self.get_logger().error("Frame is None. Unable to display the image.")
                return
        except Exception as e:
            self.get_logger().error(f"Error displaying raw image: {e}")
            return

        # Debug the size of the received image
        self.get_logger().info(f"Image shape: {frame.shape if frame is not None else 'None'}")

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.get_logger().info("Image converted to grayscale.")

        # Detect ArUco markers
        corners, ids, rejected = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )
        self.get_logger().info(f"Markers detected: {ids if ids is not None else 'None'}")

        if ids is not None and len(ids) > 0:
            self.get_logger().info(f"Detected {len(ids)} markers: {ids.flatten()}")

            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_length,
                self.camera_matrix,
                self.dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                self.get_logger().info(f"Marker ID {marker_id}: rvec={rvecs[i].flatten()}, tvec={tvecs[i].flatten()}")

                # Publish pose
                pose_msg = PoseStamped()
                pose_msg.header = Header()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'camera_frame'

                pose_msg.pose.position.x = float(tvecs[i][0][0])
                pose_msg.pose.position.y = float(tvecs[i][0][1])
                pose_msg.pose.position.z = float(tvecs[i][0][2])

                quat = self.rvec_to_quaternion(rvecs[i])
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                self.pose_pub.publish(pose_msg)
                self.get_logger().info(f"Published pose for marker ID {marker_id}.")

                # Calculate distance from the camera
                distance = np.linalg.norm(tvecs[i][0])
                self.distance_pub.publish(Float32(data=distance))
                self.get_logger().info(f"Published distance for marker ID {marker_id}: {distance:.2f} meters.")

                # Draw the marker and pose on the image
                try:
                    self.draw_axis(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i])
                    aruco.drawDetectedMarkers(frame, corners, ids)
                except Exception as e:
                    self.get_logger().error(f"Error drawing axis or markers: {e}")

            # Display the result frame
            try:
                cv2.imshow("ArUco Detection", frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().error(f"Error displaying frame: {e}")

            # Publish the annotated image
            try:
                _, buffer = cv2.imencode('.jpg', frame)
                annotated_msg = CompressedImage()
                annotated_msg.header.stamp = msg.header.stamp
                annotated_msg.format = "jpeg"
                annotated_msg.data = buffer.tobytes()
                self.image_pub.publish(annotated_msg)
                self.get_logger().info("Published annotated image.")
            except Exception as e:
                self.get_logger().error(f"Failed to publish annotated image: {e}")
        else:
            self.get_logger().info("No markers detected.")

    def draw_axis(self, image, camera_matrix, dist_coeffs, rvec, tvec, length=0.1):
        axis_points = np.float32([
            [0, 0, 0],  # Origin
            [length, 0, 0],  # X-axis
            [0, length, 0],  # Y-axis
            [0, 0, -length]  # Z-axis
        ]).reshape(-1, 3)

        img_points, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)

        origin = tuple(img_points[0].ravel().astype(int))
        x_axis = tuple(img_points[1].ravel().astype(int))
        y_axis = tuple(img_points[2].ravel().astype(int))
        z_axis = tuple(img_points[3].ravel().astype(int))

        cv2.line(image, origin, x_axis, (0, 0, 255), 2)
        cv2.line(image, origin, y_axis, (0, 255, 0), 2)
        cv2.line(image, origin, z_axis, (255, 0, 0), 2)
        cv2.circle(image, origin, 3, (0, 0, 0), -1)

    def rvec_to_quaternion(self, rvec):
        try:
            rotation = R.from_rotvec(rvec.flatten())
            return rotation.as_quat()  # Returns [x, y, z, w]
        except Exception as e:
            self.get_logger().error(f"Error converting rvec to quaternion: {e}")
            return [0.0, 0.0, 0.0, 1.0]


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArUco Detection Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
