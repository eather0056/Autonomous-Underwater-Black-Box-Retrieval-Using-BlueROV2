#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import gi
import numpy as np
import json

from sensor_msgs.msg import BatteryState
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import String
from bluerov2_interfaces.msg import Bar30
from bluerov2_interfaces.msg import Attitude

gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Controller(Node):
    g = 9.81  # m.s^-2 gravitational acceleration
    p0 = 103425  # Surface pressure in Pascal
    rho = 1000  # kg/m^3 water density

    def __init__(self):
        super().__init__("video")

        self.declare_parameter("port", 5600)

        self.port = self.get_parameter("port").value
        self._frame = None
        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.voltage = 0.0
        self.depth = 0.0
        self.depth_desired = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0

        # Font for overlay
        self.font = cv2.FONT_HERSHEY_PLAIN

        # Initialize CvBridge and publishers
        self.bridge = CvBridge()
        self.publisher_image = self.create_publisher(CompressedImage, 'camera/image_raw/compressed', 10)
        self.publisher_cam_info = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Create subscribers
        self.battery_sub = self.create_subscription(BatteryState, "/bluerov2/battery", self.battery_callback, 10)
        self.depth_desired_sub = self.create_subscription(String, "/settings/depth/status", self.depth_desired_callback, 10)
        self.bar30_sub = self.create_subscription(Bar30, "/bluerov2/bar30", self.callback_bar30, 10)
        self.attitude_sub = self.create_subscription(Attitude, "/bluerov2/attitude", self.callback_att, 10)

        # Calibration data
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

        Gst.init()
        self.run()

        # Start update loop
        self.create_timer(0.01, self.update)

    def start_gst(self, config=None):
        if not config:
            config = [
                'videotestsrc ! decodebin',
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]
        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        return self._frame

    def frame_available(self):
        return type(self._frame) != type(None)

    def run(self):
        self.start_gst([
            self.video_source,
            self.video_codec,
            self.video_decode,
            self.video_sink_conf
        ])
        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame
        return Gst.FlowReturn.OK

    def battery_callback(self, msg):
        self.voltage = round(msg.voltage, 2)

    def depth_desired_callback(self, msg):
        data = json.loads(msg.data)
        self.depth_desired = abs(data['depth_desired'])

    def callback_bar30(self, msg):
        self.depth = round((msg.press_abs * 100 - self.p0) / (self.rho * self.g), 2)

    def callback_att(self, msg):
        self.roll = round(msg.roll, 3)
        self.pitch = round(msg.pitch, 3)
        self.yaw = round(msg.yaw, 3)

    def update(self):
        if not self.frame_available():
            return

        frame = self.frame()
        img = cv2.resize(frame, (self.image_width, self.image_height), interpolation=cv2.INTER_AREA)

        self.draw_gui(img)

        # Convert OpenCV image to ROS 2 compressed image message
        compressed_image = CompressedImage()
        compressed_image.header.stamp = self.get_clock().now().to_msg()
        compressed_image.header.frame_id = "camera_frame"
        compressed_image.format = "jpeg"
        compressed_image.data = cv2.imencode('.jpg', img)[1].tobytes()

        # Publish the image and CameraInfo messages
        self.publisher_image.publish(compressed_image)
        self.publisher_cam_info.publish(self.camera_info_msg)

        # Optionally display the image using OpenCV
        cv2.imshow('BlueROV2 Camera', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def draw_gui(self, img):
        height = img.shape[0]
        width = img.shape[1]

        # Draw red dot at the center of the image
        center_x = width // 2
        center_y = height // 2
        cv2.circle(img, (center_x, center_y), 5, (0, 0, 255), -1)  # Red dot

        img = cv2.rectangle(img, (0, height - 100), (520, height), (0, 0, 0), -1)
        img = cv2.putText(img, f'Voltage: {self.voltage}V', (10, height - 70), self.font, 1.6, (255, 255, 255), 1)
        img = cv2.putText(img, f'Depth: {self.depth}m', (10, height - 45), self.font, 1.6, (255, 255, 255), 1)
        img = cv2.putText(img, f'Target Depth: {self.depth_desired}m', (10, height - 20), self.font, 1.6, (255, 255, 255), 1)
        img = cv2.putText(img, f'Pitch: {self.pitch}', (320, height - 70), self.font, 1.6, (255, 255, 255), 1)
        img = cv2.putText(img, f'Roll: {self.roll}', (320, height - 45), self.font, 1.6, (255, 255, 255), 1)
        img = cv2.putText(img, f'Yaw: {self.yaw}', (320, height - 20), self.font, 1.6, (255, 255, 255), 1)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
