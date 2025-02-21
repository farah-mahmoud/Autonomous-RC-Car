#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
import serial
import threading
import time  # Needed for sleep function
import os

class NewTeleopSerial(Node):
    def __init__(self):
        super().__init__('new_teleop_serial')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10)

        self.serial_port = self.open_serial_port()
        self.last_command_time = self.get_clock().now()
        self.should_stop = False

        # Start a thread to send stop command when no key is pressed
        self.stop_thread = threading.Thread(target=self.send_stop_signal)
        self.stop_thread.daemon = True
        self.stop_thread.start()

    def open_serial_port(self):
        """Opens the serial port safely and handles errors."""
        try:
            if os.path.exists('/dev/my_serial'):  # Check if the serial port exists
                return serial.Serial('/dev/my_serial', 74880, timeout=1, write_timeout=2)
            else:
                self.get_logger().error("Serial port /dev/my_serial not found!")
                return None  # Return None if port is unavailable
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return None  # Avoid crashes

    def listener_callback(self, msg):
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().warn("Serial port not open. Skipping write.")
            return

        linear_x = int(msg.linear.x * 100)
        angular_z = int(msg.angular.z * 100)

        command = f"{linear_x},{angular_z}\n"
        try:
            self.serial_port.write(command.encode())
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timeout! Reconnecting...")
            self.reconnect_serial()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.reconnect_serial()

        if linear_x != 0 or angular_z != 0:
            self.get_logger().info(f"Sent: {command}")

        self.last_command_time = self.get_clock().now()
        self.should_stop = False

    def send_stop_signal(self):
        while rclpy.ok():
            current_time = self.get_clock().now()
            if (current_time - self.last_command_time) > Duration(seconds=0.5):
                self.should_stop = True  

            if self.should_stop:
                stop_command = "0,0\n"
                if self.serial_port and self.serial_port.is_open:
                    try:
                        self.serial_port.write(stop_command.encode())
                        self.get_logger().info("Sent: 0,0 (Stop)")
                    except serial.SerialTimeoutException:
                        self.get_logger().warn("Serial write timeout on stop signal! Reconnecting...")
                        self.reconnect_serial()
                    except serial.SerialException as e:
                        self.get_logger().error(f"Serial error: {e}")
                        self.reconnect_serial()
                else:
                    self.get_logger().warn("Serial port not open. Cannot send stop signal.")

                self.should_stop = False  

            time.sleep(0.1)  # Prevent excessive CPU usage

    def reconnect_serial(self):
        """Tries to close and reopen the serial port if disconnected."""
        self.get_logger().info("Attempting to reconnect serial port...")
        try:
            if self.serial_port:
                self.serial_port.close()
            time.sleep(1)
            self.serial_port = self.open_serial_port()
            if self.serial_port and self.serial_port.is_open:
                self.get_logger().info("Serial port reconnected successfully!")
            else:
                self.get_logger().warn("Failed to reconnect serial port.")
        except serial.SerialException as e:
            self.get_logger().error(f"Reconnection failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NewTeleopSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
