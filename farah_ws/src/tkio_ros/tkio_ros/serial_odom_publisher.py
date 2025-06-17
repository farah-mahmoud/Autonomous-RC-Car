#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf2_ros
import math
from hardware.serial_driver import SerialReceiver
from hardware.base_driver import BaseDriver


class SerialOdomNode(Node):
    def __init__(self):
        super().__init__('serial_odom_publisher')

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Serial Receiver
        self.driver = SerialReceiver(
            msgName='serial_odom_rx',
            channel='/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_85036303232351617110-if00',
            msgID=None,
            msgIDLength=0,
            baudrate=115200,
            timeout=5
        )

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.steering_angle = 0.0
        self.wheel_pos = 0.0
        self.last_time = self.get_clock().now()
        self.last_speed = 0.0  # m/s
        self.last_yawrate = 0.0  # deg/s

        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.update)

    def update(self):
        """Read from Serial, parse 'yaw_rate=<deg/sec>, speed=<m/s>' messages and publish odom."""
        line = BaseDriver.receivedMsgsBuffer[self.driver.channel][None]
        if line is None:
            return

        try:
            decoded = line.decode().strip()
            if "yaw_rate=" in decoded and "speed=" in decoded:
                parts = decoded.split(",")
                yaw_str = parts[0].split("=")[1].strip()
                speed_str = parts[1].split("=")[1].strip()

                self.last_yawrate = float(yaw_str)  # in deg/s
                self.last_speed = float(speed_str)  # in m/s
            else:
                return

            v = self.last_speed  # m/s
            w = math.radians(self.last_yawrate)  # deg/s to rad/s

        except Exception as e:
            self.get_logger().warn(f"Failed to parse line: {decoded} ({e})")
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        # Dead reckoning update
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Update steering and wheels
        self.steering_angle = w * dt
        self.steering_angle = max(min(self.steering_angle, 0.6), -0.6)  # ±35°
        self.wheel_pos += v * dt / 0.033  # 33mm radius wheels

        # Prepare Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        # Transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        # JointState message
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = [
            'front_left_hinge_joint',
            'front_right_hinge_joint',
            'front_left_joint',
            'front_right_joint',
            'rear_left_joint',
            'rear_right_joint'
        ]
        joint_msg.position = [
            self.steering_angle,
            self.steering_angle,
            self.wheel_pos,
            self.wheel_pos,
            self.wheel_pos,
            self.wheel_pos
        ]
        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()