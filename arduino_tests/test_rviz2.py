#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ToggleControl(Node):
    def __init__(self):
        super().__init__("toggle_controller")

        # Subscriber to RViz button (assumed Bool message type)
        self.subscription = self.create_subscription(
            Bool,
            "/rviz_button",
            self.button_callback,
            10
        )

        # Publisher to control robot movement
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.is_moving = False  # Toggle state

    def button_callback(self, msg):
        if msg.data:  # If button is pressed
            self.is_moving = not self.is_moving  # Toggle state

            twist_msg = Twist()
            if self.is_moving:
                twist_msg.linear.x = 0.5  # Move forward
            else:
                twist_msg.linear.x = 0.0  # Stop

            self.cmd_pub.publish(twist_msg)
            self.get_logger().info(f"Robot {'moving' if self.is_moving else 'stopped'}")

def main(args=None):
    rclpy.init(args=args)
    node = ToggleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
