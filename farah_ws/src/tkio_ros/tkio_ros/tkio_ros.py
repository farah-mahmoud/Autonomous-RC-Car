#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# Import the SerialSender driver from your hardware library
from hardware.serial_driver import SerialSender

class TkioRos(Node):
    def __init__(self):
        super().__init__('TkioRos')

        # --- Parameters ---
        self.declare_parameter('cmd_vel_topic', '/asc/reference')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_channel', '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85036303232351617110-if00')
        self.declare_parameter('send_rate_hz', 10.0)

        # --- Read parameters ---
        topic       = self.get_parameter('cmd_vel_topic').value
        baudrate    = self.get_parameter('baudrate').value
        channel     = self.get_parameter('serial_channel').value
        rate_hz     = self.get_parameter('send_rate_hz').value
        self._min_interval = 1.0 / rate_hz

        # --- Initialize SerialSender ---
        self.driver = SerialSender(
            msgName='teleop_tx',
            channel=channel,
            msgID=None,
            msgIDLength=0,
            baudrate=baudrate,
            timeout=5
        )

        # --- State ---
        self.last_sent_time = 0.0

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            TwistStamped,
            topic,
            self.listener_callback,
            10
        )

        self.get_logger().info(f"TeleopSerialNode listening on '{topic}', serial: {channel} @ {baudrate}bps")

    def listener_callback(self, msg: TwistStamped):
        now = time.time()
        if (now - self.last_sent_time) < self._min_interval:
            return
        self.last_sent_time = now

        # Convert to integer commands [-100..100]
        linear_pct = int(max(min(msg.twist.linear.x, 1.0), -1.0) * 100)
        # Assuming angular.z is normalized [-1..1]
        steering_pct = int(max(min(msg.twist.angular.z, 1.0), -1.0) * 100)

        # Build payload e.g. "50,-20\n"
        payload = f"{linear_pct},{steering_pct}\n".encode('ascii')

        # Send via SerialSender
        status = self.driver.send(payload)
        if status == 0:
            self.get_logger().info(f"Sent: {payload.strip()}")
        else:
            self.get_logger().error(f"Failed to send payload, status={status}")


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down TeleopSerialNode...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
