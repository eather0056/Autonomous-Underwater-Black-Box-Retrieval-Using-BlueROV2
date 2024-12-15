#!/usr/bin/env python3

import sys
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import CompressedImage, BatteryState
from bluerov2_interfaces.msg import Bar30, Attitude
from std_msgs.msg import UInt16, Float64, Bool, String, Float32

from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QGridLayout,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QLabel,
    QSlider,
    QPushButton,
    QSizePolicy,
    QGroupBox,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap


class GUIController(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, "gui_controller")
        QMainWindow.__init__(self)

        # Declare and retrieve ROS parameters
        self.declare_parameter("pwm_gain", 200)
        self.declare_parameter("gain_depth", 0.1)
        self.pwm_gain = self.get_parameter("pwm_gain").value
        self.gain_depth = self.get_parameter("gain_depth").value

        # Control variables
        self.control_mode = "manual"
        self.aruco_distance = 0.0
        self.voltage = 0.0
        self.depth = 0.0
        self.depth_desired = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.roll = 0.0

        # Depth control variables
        self.current_depth = 0.0

        # Constants for bar30 callback:
        self.rho = 997   # water density (kg/m^3)
        self.g = 9.81
        self.p0 = 101300

        # We'll store the latest decoded frames here
        self.normal_image_cv = None
        self.detected_image_cv = None

        # ----------------- Publishers -----------------
        self.pwm_gain_pub = self.create_publisher(UInt16, "/settings/pwm_gain", 10)
        self.depth_controller_pub = self.create_publisher(Float64, "/settings/depth/set_depth", 10)
        self.mode_pub = self.create_publisher(String, "/settings/control_mode", 10)
        self.arm_pub = self.create_publisher(Bool, "/bluerov2/arm", 10)
        self.lights_pub = self.create_publisher(UInt16, "/bluerov2/rc/lights", 10)
        self.gripper_pub = self.create_publisher(UInt16, "/bluerov2/rc/gripper", 10)
        self.forward_pub = self.create_publisher(UInt16, "/bluerov2/rc/forward", 10)
        self.lateral_pub = self.create_publisher(UInt16, "/bluerov2/rc/lateral", 10)
        self.yaw_pub = self.create_publisher(UInt16, "/bluerov2/rc/yaw", 10)

        self.yaw_kp_pub = self.create_publisher(Float64, "/settings/yaw/kp", 10)
        self.yaw_kd_pub = self.create_publisher(Float64, "/settings/yaw/kd", 10)
        self.forward_kp_pub = self.create_publisher(Float64, "/settings/forward/kp", 10)
        self.forward_kd_pub = self.create_publisher(Float64, "/settings/forward/kd", 10)
        self.lateral_kp_pub = self.create_publisher(Float64, "/settings/lateral/kp", 10)
        self.lateral_kd_pub = self.create_publisher(Float64, "/settings/lateral/kd", 10)
        self.throttle_kp_pub = self.create_publisher(Float64, "/settings/throttle/kp", 10)
        self.throttle_kd_pub = self.create_publisher(Float64, "/settings/throttle/kd", 10)
        self.forward_offset_pub = self.create_publisher(Float64, "/setings/forward_Offset_correction", 10)
        self.yaw_offset_pub = self.create_publisher(Float64, "/setings/yaw_Offset_correction", 10)
        self.lateral_offset_pub = self.create_publisher(Float64, "/setings/lateral_Offset_correction", 10)

        # ----------------- Subscribers -----------------
        # Subscribe to CompressedImage topics

        self.create_subscription(CompressedImage, "/camera/image_raw/compressed", self.normal_image_callback, 10)
        self.create_subscription(CompressedImage, "/aruco_detected_image", self.detected_image_callback, 10)
        self.create_subscription(Float32, "/aruco_distance", self.update_aruco_distance, 10)
        self.create_subscription(BatteryState, "/bluerov2/battery", self.battery_callback, 10)
        self.create_subscription(String, "/settings/depth/status", self.depth_desired_callback, 10)
        self.create_subscription(Bar30, "/bluerov2/bar30", self.callback_bar30, 10)
        self.create_subscription(Attitude, "/bluerov2/attitude", self.callback_att, 10)

        # -------------------- PyQt5 GUI Setup --------------------
        self.setWindowTitle("BlueROV2 GUI Controller")
        self.resize(1200, 800)

        # Central widget
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        main_layout = QGridLayout(central_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        main_layout.setSpacing(10)

        # Image frame
        image_frame = QFrame()
        image_frame_layout = QGridLayout(image_frame)
        main_layout.addWidget(image_frame, 0, 0, 1, 2)

        label_normal = QLabel("Normal Image")
        label_normal.setStyleSheet("color: white; background-color: black; font: bold 12px;")
        image_frame_layout.addWidget(label_normal, 0, 0)

        self.normal_image_label = QLabel()
        self.normal_image_label.setStyleSheet("background-color: black;")
        self.normal_image_label.setFixedSize(640, 480)
        self.normal_image_label.setScaledContents(True)
        # self.normal_image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        image_frame_layout.addWidget(self.normal_image_label, 1, 0)

        label_detected = QLabel("Detected Image")
        label_detected.setStyleSheet("color: white; background-color: black; font: bold 12px;")
        image_frame_layout.addWidget(label_detected, 0, 1)

        self.detected_image_label = QLabel()
        self.detected_image_label.setStyleSheet("background-color: black;")
        self.detected_image_label.setFixedSize(640, 480)
        self.detected_image_label.setScaledContents(True)
        # self.detected_image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        image_frame_layout.addWidget(self.detected_image_label, 1, 1)

        # Control frame
        control_frame = QFrame()
        control_frame_layout = QGridLayout(control_frame)
        control_frame.setStyleSheet("background-color: #f0f0f0;")
        main_layout.addWidget(control_frame, 1, 0)

        # Button frame
        button_frame = QFrame()
        button_frame_layout = QGridLayout(button_frame)
        button_frame.setStyleSheet("background-color: #f0f0f0;")
        main_layout.addWidget(button_frame, 1, 1)

        distance_title = QLabel("ArUco Distance (m)")
        distance_title.setStyleSheet("font: bold 12px; background-color: #f0f0f0;")
        control_frame_layout.addWidget(distance_title, 6, 0)

        self.distance_label = QLabel(f"Distance: {self.aruco_distance:.2f} m")
        self.distance_label.setStyleSheet("color: blue; font: 12px; background-color: #f0f0f0;")
        control_frame_layout.addWidget(self.distance_label, 6, 1)

        # Depth slider
        depth_label = QLabel("Depth Control")
        depth_label.setStyleSheet("font: bold 12px; background-color: #f0f0f0;")
        control_frame_layout.addWidget(depth_label, 7, 0)

        self.depth_slider = QSlider(Qt.Horizontal)
        self.depth_slider.setMinimum(int(-10.0 * 10))  # -100
        self.depth_slider.setMaximum(int(0.0 * 10))    # 0
        self.depth_slider.setSingleStep(1)
        self.depth_slider.setValue(int(self.current_depth * 10))
        self.depth_slider.setStyleSheet("background-color: lightgreen;")
        self.depth_slider.valueChanged.connect(self.on_depth_slider_changed)
        control_frame_layout.addWidget(self.depth_slider, 7, 1)

        # PWM slider
        pwm_label = QLabel("PWM Gain")
        pwm_label.setStyleSheet("font: bold 12px; background-color: #f0f0f0;")
        control_frame_layout.addWidget(pwm_label, 8, 0)

        self.pwm_slider = QSlider(Qt.Horizontal)
        self.pwm_slider.setMinimum(100)
        self.pwm_slider.setMaximum(300)
        self.pwm_slider.setValue(self.pwm_gain)
        self.pwm_slider.setStyleSheet("background-color: lightblue;")
        self.pwm_slider.valueChanged.connect(self.on_pwm_slider_changed)
        control_frame_layout.addWidget(self.pwm_slider, 8, 1)

        # PID & offset sliders
        self.add_slider("Yaw KP", control_frame_layout, row=0, col=0, from_=0.0, to=100.0, default=40, pub=self.yaw_kp_pub)
        self.add_slider("Yaw KD", control_frame_layout, row=0, col=2, from_=0.0, to=50.0, default=20, pub=self.yaw_kd_pub)
        self.add_slider("Forward KP", control_frame_layout, row=1, col=0, from_=0.0, to=100.0, default=79, pub=self.forward_kp_pub)
        self.add_slider("Forward KD", control_frame_layout, row=1, col=2, from_=0.0, to=50.0, default=30, pub=self.forward_kd_pub)
        self.add_slider("Lateral KP", control_frame_layout, row=2, col=0, from_=0.0, to=50.0, default=30, pub=self.lateral_kp_pub)
        self.add_slider("Lateral KD", control_frame_layout, row=2, col=2, from_=0.0, to=50.0, default=5, pub=self.lateral_kd_pub)
        self.add_slider("Throttle KP", control_frame_layout, row=3, col=0, from_=0.0, to=300.0, default=200, pub=self.throttle_kp_pub)
        self.add_slider("Throttle KD", control_frame_layout, row=3, col=2, from_=0.0, to=100.0, default=30, pub=self.throttle_kd_pub)
        self.add_slider("Forward Offset", control_frame_layout, row=4, col=0, from_=0.0, to=2.0, default=0.7, pub=self.forward_offset_pub)
        self.add_slider("Yaw Offset", control_frame_layout, row=4, col=2, from_=-20.0, to=20.0, default=-10, pub=self.yaw_offset_pub)
        self.add_slider("Lateral Offset", control_frame_layout, row=5, col=0, from_=-2.0, to=2.0, default=0.4, pub=self.lateral_offset_pub)

        # Movement buttons
        movement_buttons = [
            ("Forward", self.forward_pub, 1600, "lightgreen"),
            ("Backward", self.forward_pub, 1400, "lightgreen"),
            ("Left", self.lateral_pub, 1400, "lightblue"),
            ("Right", self.lateral_pub, 1600, "lightblue"),
            ("Yaw Left", self.yaw_pub, 1400, "yellow"),
            ("Yaw Right", self.yaw_pub, 1600, "yellow"),
        ]
        action_buttons = [
            ("Lights Up", lambda: self.adjust_lights("up"), "orange"),
            ("Lights Down", lambda: self.adjust_lights("down"), "orange"),
            ("Gripper Open", lambda: self.adjust_gripper("open"), "pink"),
            ("Gripper Close", lambda: self.adjust_gripper("close"), "pink"),
            ("Arm", self.arm, "red"),
            ("Disarm", self.disarm, "red"),
            ("Manual Mode", lambda: self.switch_mode("manual"), "purple"),
            ("ArUco Mode", lambda: self.switch_mode("aruco"), "purple"),
        ]
        for i, (text, pub, val, color) in enumerate(movement_buttons):
            btn = QPushButton(text)
            btn.setStyleSheet(f"background-color: {color}; font: 10px;")
            btn.setFixedWidth(130)
            btn.pressed.connect(lambda _, p=pub, v=val: self.send_movement_command(p, v))
            btn.released.connect(lambda _, p=pub: self.stop_movement(p))
            button_frame_layout.addWidget(btn, i // 6, i % 6)

        base_row = len(movement_buttons) // 6
        for i, (text, callback, color) in enumerate(action_buttons):
            btn = QPushButton(text)
            btn.setStyleSheet(f"background-color: {color}; font: 10px;")
            btn.setFixedWidth(130)
            btn.clicked.connect(callback)
            row = base_row + (i // 6)
            col = i % 6
            button_frame_layout.addWidget(btn, row, col)

        # Data display frame
        data_frame = QFrame()
        data_frame_layout = QGridLayout(data_frame)
        data_frame.setStyleSheet("background-color: #f0f0f0;")
        main_layout.addWidget(data_frame, 2, 0, 1, 2)

        self.voltage_label = QLabel(f"Voltage: {self.voltage} V")
        self.voltage_label.setStyleSheet("font: 12px; background-color: #f0f0f0;")
        data_frame_layout.addWidget(self.voltage_label, 0, 0)

        self.depth_label = QLabel(f"Depth: {self.depth} m")
        self.depth_label.setStyleSheet("font: 12px; background-color: #f0f0f0;")
        data_frame_layout.addWidget(self.depth_label, 0, 1)

        self.depth_desired_label = QLabel(f"Target Depth: {self.depth_desired} m")
        self.depth_desired_label.setStyleSheet("font: 12px; background-color: #f0f0f0;")
        data_frame_layout.addWidget(self.depth_desired_label, 0, 2)

        self.pitch_label = QLabel(f"Pitch: {self.pitch}")
        self.pitch_label.setStyleSheet("font: 12px; background-color: #f0f0f0;")
        data_frame_layout.addWidget(self.pitch_label, 1, 0)

        self.roll_label = QLabel(f"Roll: {self.roll}")
        self.roll_label.setStyleSheet("font: 12px; background-color: #f0f0f0;")
        data_frame_layout.addWidget(self.roll_label, 1, 1)

        self.yaw_label = QLabel(f"Yaw: {self.yaw}")
        self.yaw_label.setStyleSheet("font: 12px; background-color: #f0f0f0;")
        data_frame_layout.addWidget(self.yaw_label, 1, 2)

        # Timer to refresh images in the GUI
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self.update_image_labels)
        self.refresh_timer.start(50)  # ~20 FPS

        self.show()

    # ----------------- CompressedImage callbacks -----------------
    def normal_image_callback(self, msg):
        """Decode /camera/image_raw/compressed -> OpenCV RGB."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_bgr is not None:
                self.normal_image_cv = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().error(f"Failed to decode normal image: {e}")

    def detected_image_callback(self, msg):
        """Decode /aruco_detected_image -> OpenCV RGB."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_bgr is not None:
                self.detected_image_cv = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().error(f"Failed to decode detected image: {e}")

    def update_aruco_distance(self, msg):
        self.aruco_distance = msg.data
        self.distance_label.setText(f"Distance: {self.aruco_distance:.2f} m")

    def update_image_labels(self):
        # Update Normal Image
        if self.normal_image_cv is not None:
            h, w, c = self.normal_image_cv.shape
            bytes_per_line = c * w
            qt_img = QImage(self.normal_image_cv.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.normal_image_label.setPixmap(QPixmap.fromImage(qt_img))

        # Update Detected Image
        if self.detected_image_cv is not None:
            h, w, c = self.detected_image_cv.shape
            bytes_per_line = c * w
            qt_img = QImage(self.detected_image_cv.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.detected_image_label.setPixmap(QPixmap.fromImage(qt_img))

    # ----------------- Sliders & Param Updates -------------------
    def add_slider(self, label, layout, row, col, from_, to, default, pub):
        title = QLabel(label)
        title.setStyleSheet("font: bold 12px; background-color: #f0f0f0;")
        layout.addWidget(title, row, col)

        slider = QSlider(Qt.Horizontal)
        slider.setStyleSheet("background-color: lightblue;")
        slider.setMinimum(int(from_ * 10))
        slider.setMaximum(int(to * 10))
        slider.setValue(int(default * 10))
        slider.setSingleStep(1)
        slider.valueChanged.connect(lambda val: self.on_generic_slider_changed(val, pub))
        layout.addWidget(slider, row, col + 1)

    def on_generic_slider_changed(self, val, publisher):
        float_val = val / 10.0
        msg = Float64()
        msg.data = float_val
        publisher.publish(msg)
        self.get_logger().info(f"Updated {publisher.topic_name} to {float_val}")

    def on_depth_slider_changed(self, val):
        depth_meters = val / 10.0
        msg = Float64()
        msg.data = depth_meters
        self.depth_controller_pub.publish(msg)

    def on_pwm_slider_changed(self, val):
        self.pwm_gain = val
        self.pwm_gain_pub.publish(UInt16(data=self.pwm_gain))

    # ----------------- Movement & Action Buttons -----------------
    def send_movement_command(self, publisher, value):
        msg = UInt16()
        msg.data = value
        publisher.publish(msg)

    def stop_movement(self, publisher):
        msg = UInt16()
        msg.data = 1500
        publisher.publish(msg)

    def arm(self):
        self.arm_pub.publish(Bool(data=True))

    def disarm(self):
        self.arm_pub.publish(Bool(data=False))

    def adjust_lights(self, direction):
        msg = UInt16(data=1900 if direction == "up" else 1100)
        self.lights_pub.publish(msg)

    def adjust_gripper(self, direction):
        msg = UInt16(data=1900 if direction == "open" else 1100)
        self.gripper_pub.publish(msg)

    def switch_mode(self, mode):
        self.mode_pub.publish(String(data=mode))

    # ----------------- Other ROS Callbacks -----------------------
    def battery_callback(self, msg):
        self.voltage = round(msg.voltage, 2)
        self.voltage_label.setText(f"Voltage: {self.voltage} V")

    def depth_desired_callback(self, msg):
        data = json.loads(msg.data)
        self.depth_desired = abs(data['depth_desired'])
        self.depth_desired_label.setText(f"Target Depth: {self.depth_desired} m")

    def callback_bar30(self, msg):
        water_column_pressure = (msg.press_abs * 100) - self.p0
        depth_m = water_column_pressure / (self.rho * self.g)
        self.depth = round(depth_m, 2)
        self.depth_label.setText(f"Depth: {self.depth} m")

    def callback_att(self, msg):
        self.roll = round(msg.roll, 3)
        self.pitch = round(msg.pitch, 3)
        self.yaw = round(msg.yaw, 3)
        self.roll_label.setText(f"Roll: {self.roll}")
        self.pitch_label.setText(f"Pitch: {self.pitch}")
        self.yaw_label.setText(f"Yaw: {self.yaw}")

    # ----------------- Shutdown -----------------
    def closeEvent(self, event):
        self.destroy_node()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui_node = GUIController()

    # Use a QTimer to spin rclpy without blocking the GUI
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(gui_node, timeout_sec=0.01))
    timer.start(10)  # spin at ~100 Hz

    try:
        app.exec_()
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
