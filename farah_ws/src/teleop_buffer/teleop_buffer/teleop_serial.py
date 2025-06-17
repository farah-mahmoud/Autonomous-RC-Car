import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading

class TeleopSerial(Node):
    def __init__(self):
        super().__init__('teleop_serial')
        
        # ROS2 Subscription
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10)
        
        # Initialize Serial Connection
        self.serial_port = self.open_serial()
        self.last_sent_time = time.time()

        # Start keep-alive thread
        threading.Thread(target=self.keep_serial_alive, daemon=True).start()

    def open_serial(self):
        """Tries to open the serial port and returns the Serial object."""
        try:
            ser = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_8503532323435151B1C2-if00', 115200, timeout=0.1)
            self.get_logger().info("Serial port opened successfully.")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return None

    def keep_serial_alive(self):
        """Continuously reads from the serial port to prevent freezing."""
        while rclpy.ok():
            try:
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.readline()  # Read and discard incoming data
            except serial.SerialException:
                self.get_logger().warn("Serial read error, attempting to reconnect...")
                self.serial_port = self.open_serial()

    def listener_callback(self, msg):
        """Handles incoming Twist messages and sends data over serial."""
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn("Serial port is closed, attempting to reopen...")
            self.serial_port = self.open_serial()
            if self.serial_port is None:
                return

        current_time = time.time()
        if current_time - self.last_sent_time < 0.1:  # Send every 100ms
            return

        self.last_sent_time = current_time
        linear_x = int(msg.linear.x * 100)
        angular_z = int(msg.angular.z * 100)
        command = f"{linear_x},{angular_z}\n"

        try:
            self.serial_port.reset_input_buffer()  # Flush input buffer
            self.serial_port.reset_output_buffer()  # Flush output buffer
            self.serial_port.write(command.encode())
            self.serial_port.flush()
            self.get_logger().info(f"Sent: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            self.serial_port = None

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
