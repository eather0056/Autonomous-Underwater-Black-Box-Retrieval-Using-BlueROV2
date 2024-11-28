import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import gi
import numpy as np
import cv2

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class GstCameraPublisher(Node):
    def __init__(self):
        super().__init__('bluerov2_camera_publisher')

        # Declare ROS2 parameter for port
        self.declare_parameter("port", 5600)
        self.port = self.get_parameter("port").value

        # Initialize ROS2 publishers
        self.publisher_image = self.create_publisher(Image, 'camera/image_raw', 10)
        self.publisher_compressed_image = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        self.bridge = CvBridge()

        # Initialize GStreamer
        Gst.init(None)
        self.pipeline = Gst.parse_launch(
            f'udpsrc port={self.port} ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink emit-signals=true sync=false'
        )
        self.appsink = self.pipeline.get_by_name("appsink0")
        if not self.appsink:
            self.get_logger().error("Failed to initialize appsink")
            raise RuntimeError("Appsink element not found in GStreamer pipeline")

        # Start the GStreamer pipeline
        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(f"GStreamer pipeline started on port {self.port}")

        # Timer to fetch and publish frames
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 Hz

    def gst_to_opencv(self, sample):
        """Convert GStreamer sample to OpenCV frame."""
        buf = sample.get_buffer()
        caps = sample.get_caps()

        # Extract width and height
        width = caps.get_structure(0).get_value('width')
        height = caps.get_structure(0).get_value('height')

        # Check buffer size
        expected_size = width * height * 3
        if buf.get_size() != expected_size:
            self.get_logger().error(f"Buffer size mismatch: {buf.get_size()} != {expected_size}")
            return None

        # Convert buffer to numpy array
        array = np.ndarray(
            (height, width, 3),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8
        )
        return array

    def publish_frame(self):
        """Fetch frames from GStreamer and publish as ROS2 Image and CompressedImage messages."""
        try:
            sample = self.appsink.emit("pull-sample")
            if not sample:
                self.get_logger().warn("No sample received from GStreamer pipeline")
                return

            frame = self.gst_to_opencv(sample)
            if frame is None:
                self.get_logger().warn("Failed to convert GStreamer sample to OpenCV frame")
                return

            # Publish raw image
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_image.publish(image_msg)

            # Publish compressed image
            compressed_image_msg = CompressedImage()
            compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_image_msg.header.frame_id = "camera_frame"
            compressed_image_msg.format = "jpeg"
            compressed_image_msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
            self.publisher_compressed_image.publish(compressed_image_msg)

            self.get_logger().info("Published frame (raw and compressed)")

        except Exception as e:
            self.get_logger().error(f"Error while publishing frame: {e}")

    def destroy_node(self):
        """Cleanup resources on node destruction."""
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GstCameraPublisher()
        rclpy.spin(node)
    except RuntimeError as e:
        rclpy.logging.get_logger("gst_camera_publisher").error(f"Node initialization failed: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
