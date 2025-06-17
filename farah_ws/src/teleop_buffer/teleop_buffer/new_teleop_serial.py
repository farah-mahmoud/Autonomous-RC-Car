#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from hardware.serial_driver import SerialSender

class TeleopSerialNode(Node):
    def __init__(self):
        super().__init__('teleop_serial')

        # Subscribe to Twist messages
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

        self.last_sent_time = time.time()

    def listener_callback(self, msg: Twist):
        now = time.time()
        if now - self.last_sent_time < 0.1:
            return
        self.last_sent_time = now

        # Convert to integer commands
        linear_x = int(msg.linear.x * 100)
        angular_z = int(msg.angular.z * 100)
        payload = f"{linear_x},{angular_z}\n".encode()

        status = self.driver.send(payload)
        if status == 0:
            self.get_logger().info(f"Sent command: {payload}")
        else:
            self.get_logger().error("Failed to send serial command")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
