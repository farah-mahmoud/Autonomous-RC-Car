#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ToggleControl:
    def __init__(self):
        rospy.init_node("toggle_controller")

        # Subscriber to RViz button (assumed Bool message type)
        rospy.Subscriber("/rviz_button", Bool, self.button_callback)

        # Publisher to control robot movement
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

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

if __name__ == "__main__":
    ToggleControl()
    rospy.spin()
