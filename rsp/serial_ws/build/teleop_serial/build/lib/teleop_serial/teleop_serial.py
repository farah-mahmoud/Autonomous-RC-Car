#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import sys
import termios
import tty

class TeleopSerialNode(Node):
    def __init__(self):
        super().__init__("teleop_serial")

        # Open serial connection to Arduino (TX/RX UART)
        self.ser = serial.Serial("/dev/serial0", 115200, timeout=1)

        self.get_logger().info("Teleop Serial Node Started! Use W/S/A/D/X/Space to control.")

        self.run()

    def get_key(self):
        """ Reads keyboard input from terminal """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """ Main loop for handling keyboard input and sending serial commands """
        while True:
            key = self.get_key()
            if key == "\x03":  # Ctrl+C to exit
                self.get_logger().info("Exiting teleop_serial node...")
                break

            self.ser.write(key.encode())  # Send key press to Arduino

            # Read and print Arduino feedback
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                self.get_logger().info(f"Arduino: {response}")

        self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerialNode()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

