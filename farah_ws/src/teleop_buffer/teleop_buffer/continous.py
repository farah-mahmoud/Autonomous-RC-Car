import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
import threading

class TeleopSerial(Node):
    def __init__(self):
        super().__init__('continous')

        # ROS2 Subscription
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10)

        # Initialize Serial Connection
        self.serial_port = self.open_serial()
        self.last_received_time = time.time()
        self.stop_timeout = 0.4  # 200ms timeout to stop if no new commands arrive
        self.last_sent_command = None  # Track last sent command to avoid resending same data

        self.active_command = False  # Flag to track if a command has been issued

        # Start watchdog thread
        threading.Thread(target=self.watchdog_loop, daemon=True).start()

    def open_serial(self):
        """Tries to open the serial port and returns the Serial object."""
        while True:
            try:
                ser = serial.Serial('/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85036303232351617110-if00', 115200, timeout=0.1)
                self.get_logger().info("Serial port opened successfully.")
                return ser
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port: {e}, retrying in 1s...")
                time.sleep(1)

    def listener_callback(self, msg):
        """Handles incoming Twist messages and sends data over serial."""
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warn("Serial port is closed, attempting to reopen...")
            self.serial_port = self.open_serial()

        if msg.linear.x == 0.0 and msg.angular.z == 0.0:
            self.active_command = False  # No movement
        else:
            self.active_command = True  # Movement command received

        self.last_received_time = time.time()  # Update last command time
        self.send_velocity(msg.linear.x, msg.angular.z)

    def send_velocity(self, linear_x, angular_z):
        """Formats and sends velocity commands via serial."""
        linear_x = int(linear_x * 100)
        angular_z = int(angular_z * 100)
        command = f"{linear_x},{angular_z}\n"

        if command == self.last_sent_command:
            return  # Skip sending if the command hasn't changed

        try:
            self.serial_port.write(command.encode())
            self.serial_port.flush()
            self.last_sent_command = command  # Store last sent command
            self.get_logger().info(f"Sent: {command}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")
            self.serial_port = None  # Mark serial as closed

    def watchdog_loop(self):
        """Stops the car if no input is received within timeout."""
        while rclpy.ok():
            time.sleep(0.05)  # Check every 50ms
            if self.active_command and time.time() - self.last_received_time > self.stop_timeout:
                self.send_velocity(0, 0)  # Send stop command if no new input
                self.active_command = False  # Mark command as inactive

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
