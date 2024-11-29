import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Header

class MavlinkPressureNode(Node):
    def __init__(self):
        super().__init__('mavlink_pressure_node')

        self.declare_parameter('port', 14555)  # Default MAVLink UDP port
        port = self.get_parameter('port').value

        self.pressure_publisher = self.create_publisher(FluidPressure, '/bluerov2/scaled_pressure2', 10)

        self.get_logger().info(f"Listening for MAVLink messages on UDP port {port}")
        self.connection = mavutil.mavlink_connection(f'udp:0.0.0.0:{port}')

        self.timer = self.create_timer(0.01, self.read_mavlink)  # 10 ms timer

    def read_mavlink(self):
        try:
            message = self.connection.recv_match(blocking=False)
            if not message:
                return

            # Log all received messages for debugging
            self.get_logger().info(f"Received MAVLink message: {message.get_type()}")

            if message.get_type() == 'SCALED_PRESSURE2':
                pressure = message.press_abs  # Pressure in hPa
                self.get_logger().info(f"SCALED_PRESSURE2: {pressure:.2f} hPa")

                # Create FluidPressure message
                pressure_msg = FluidPressure()
                pressure_msg.header = Header()
                pressure_msg.header.stamp = self.get_clock().now().to_msg()
                pressure_msg.header.frame_id = 'base_link'
                pressure_msg.fluid_pressure = pressure * 100.0  # Convert hPa to Pa
                pressure_msg.variance = 0.0

                self.pressure_publisher.publish(pressure_msg)

        except Exception as e:
            self.get_logger().error(f"Error reading MAVLink message: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MavlinkPressureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
