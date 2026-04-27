import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

import serial


class GPSToSerial(Node):
    def __init__(self):
        super().__init__('RoverCoordinateToAntennaBridge')

        # ===============================
        # PARAMETERS (edit as needed)
        # ===============================
        # self.serial_port = '/dev/tty.usbserial-120'   # MacBook Air
        self.serial_port = '/dev/ttyUSB0'   # Linux
        self.baudrate = 115200

        # ===============================
        # SERIAL SETUP
        # ===============================
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Serial connected: {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            self.ser = None

        # ===============================
        # ROS SUBSCRIBER
        # ===============================
        self.subscription = self.create_subscription(
            NavSatFix,
            '/nav_sat',
            self.gps_callback,
            10
        )
        self.get_logger().info("Node initiated")

    def gps_callback(self, msg: NavSatFix):
        lat = msg.latitude
        lon = msg.longitude

        # Format string
        data = f"lat:{lat:.7f},lon:{lon:.7f}\n"

        # Send via serial
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data.encode('utf-8'))
                self.get_logger().info(f"Sent: {data.strip()}")
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")
        else:
            self.get_logger().warn("Serial not available")

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSToSerial()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()