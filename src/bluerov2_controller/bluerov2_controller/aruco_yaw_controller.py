#!/usr/bin/env python3
import rclpy
import json
import math
from rclpy.node import Node
import bluerov2_controller.pid as pid

from bluerov2_interfaces.msg import Attitude, PID
from std_msgs.msg import UInt16, Bool, String
from geometry_msgs.msg import PoseStamped


class Controller(Node):

    def __init__(self):
        super().__init__("yaw_controller")

        # Setup default parameters
        self.declare_parameter("yaw_desired", 0)
        self.declare_parameter("pwm_max", 1750)
        self.declare_parameter("pwm_neutral", 1500)
        self.declare_parameter("kp", 200)
        self.declare_parameter("kd", 50)
        self.declare_parameter("enable", True)

        self.attitude = [0, 0, 0, 0, 0, 0]  # [ROLL, PITCH, YAW, ROLLSPEED, PITCHSPEED, YAWSPEED]
        self.pwm_max = self.get_parameter("pwm_max").value  # Maximum PWM value
        self.pwm_neutral = self.get_parameter("pwm_neutral").value  # Neutral PWM value
        self.yaw_desired = self.get_parameter("yaw_desired").value  # Desired pitch setpoint
        self.KP = self.get_parameter("kp").value  # Proportional gain constant
        self.KD = self.get_parameter("kd").value  # Derivative gain constant

        self.enable = self.get_parameter("enable").value

        # Create subscribers
        self.attitude_sub = self.create_subscription(Attitude, "/bluerov2/attitude", self.callback_att, 10)
        self.setYaw_sub = self.create_subscription(UInt16, "/settings/yaw/set_yaw", self.callback_set_yaw, 10)
        self.setPID_sub = self.create_subscription(PID, "/settings/yaw/set_pid", self.callback_set_pid, 10)
        self.setEnable_sub = self.create_subscription(Bool, "/settings/yaw/set_enable", self.callback_set_enable, 10)
        self.pose_sub = self.create_subscription(PoseStamped, "/aruco_pose", self.pose_callback, 10)

        # Create publishers
        self.yaw_pub = self.create_publisher(UInt16, "/bluerov2/rc/ar/yaw", 10)
        self.status_pub = self.create_publisher(String, '/settings/yaw/status', 10)

        self.marker_yaw = None  # To store the desired yaw from marker alignment

        self.get_logger().info('controller has been successfully configured!')

        # Start update loop
        self.create_timer(0.04, self.calculate_pwm)

    def callback_att(self, msg):
        self.attitude = [msg.roll,
                         msg.pitch,
                         msg.yaw,
                         msg.rollspeed,
                         msg.pitchspeed,
                         msg.yawspeed]

    def callback_set_pid(self, msg):
        """Read data from '/settings/yaw/set_pid'"""
        if msg.pwm_max != 65535:
            if msg.pwm_max < 1500:
                self.pwm_max = 1500
            else:
                self.pwm_max = msg.pwm_max

        self.KP = msg.kp if not msg.kp == 65535 else self.KP
        self.KD = msg.kd if not msg.kd == 65535 else self.KD

    def callback_set_yaw(self, msg):
        """Read data from '/settings/yaw/set_yaw'"""
        self.yaw_desired = msg.data

    def callback_set_enable(self, msg):
        """Read data from '/settings/yaw/set_enable'"""
        self.enable = msg.data

    def pose_callback(self, msg):
        """
        Callback for ArUco pose. Computes the yaw to align with the marker.
        """
        # Compute yaw from pose
        lateral_offset = msg.pose.position.x
        forward_offset = msg.pose.position.z

        # Calculate desired yaw to face the marker
        self.marker_yaw = math.atan2(lateral_offset, forward_offset)
        self.get_logger().info(f"Marker yaw updated to: {math.degrees(self.marker_yaw):.2f} degrees")

    def update_status(self):
        msg = String()
        data = {
            "type": "yaw_controller",
            "enable": self.enable,
            "kp": self.KP,
            "kd": self.KD,
            "pwm_max": self.pwm_max,
            "pwm_neutral": self.pwm_neutral,
            "yaw_desired": self.yaw_desired,
        }
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)

    def control(self, yaw, yawspeed):
        """
        Compute control input using the PID controller.
        """
        desired_yaw = self.marker_yaw if self.marker_yaw is not None else self.yaw_desired
        yaw_error = pid.sawtooth(yaw - desired_yaw)
        return self.KP * yaw_error + self.KD * yawspeed

    def calculate_pwm(self):
        """
        Calculate the PWM value for yaw control and publish it.
        """
        msg = UInt16()

        if self.enable:
            yaw = self.attitude[2]
            yawspeed = self.attitude[5]
            u = self.control(yaw, yawspeed)
            pwm = self.pwm_neutral - u

            pwm = pid.saturation(pwm, self.pwm_neutral, self.pwm_max)
            msg.data = pwm
        else:
            msg.data = self.pwm_neutral

        self.yaw_pub.publish(msg)
        self.update_status()


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
