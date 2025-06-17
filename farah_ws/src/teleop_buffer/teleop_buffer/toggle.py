#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from hardware.serial_driver import SerialSender

class TeleopSerialNode(Node):
    def __init__(self):
        super().__init__('teleop_serial')

        # Subscribe to joystick-commanded Twist messages
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )

        # Initialize SerialSender driver
        self.driver = SerialSender(
            msgName='teleop_tx',
            channel='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85036303232351617110-if00',
            msgID=None,
            msgIDLength=0,
            baudrate=115200,
            timeout=5
        )

        # Time and payload tracking
        self.last_received_time = self.get_clock().now()
        self.last_payload = b"0,0\n"

        # Timer to periodically send commands (every 0.1s)
        self.create_timer(0.1, self.timer_callback)

    def listener_callback(self, msg: Twist):
        # Convert to integer commands
        linear_x = int(msg.linear.x * 100)
        angular_z = int(msg.angular.z * 100)
        payload_str = f"{linear_x},{angular_z}\n"
        # Store the latest payload and received time
        self.last_payload = payload_str.encode()
        self.last_received_time = self.get_clock().now()

    def timer_callback(self):
        now = self.get_clock().now()
        # If no new message within 0.1s, send zero command
        elapsed = (now - self.last_received_time).nanoseconds * 1e-9
        if elapsed > 0.4:
            payload = b"0,0\n"
        else:
            payload = self.last_payload

        status = self.driver.send(payload)
        if status == 1:
            self.get_logger().error("Failed to send serial command")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()