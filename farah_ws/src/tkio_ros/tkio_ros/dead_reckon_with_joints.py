#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class DeadReckonWithJoints(Node):
    def __init__(self):
        super().__init__('dead_reckon_with_joints')

        # --- Parameters ---
        self.declare_parameter('wheelbase', 0.26)
        self.declare_parameter('rear_radius', 0.033)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('max_steer_angle', 0.367)    # rad
        self.declare_parameter('update_rate', 50.0)         # Hz

        # --- Read parameters ---
        self.wheelbase       = self.get_parameter('wheelbase').value
        self.rear_radius     = self.get_parameter('rear_radius').value
        self.cmd_vel_topic   = self.get_parameter('cmd_vel_topic').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        rate_hz              = self.get_parameter('update_rate').value

        # --- State variables ---
        self.v = 0.0
        self.delta = 0.0
        self.current_delta = 0.0
        self.max_steer_rate = 1.0  # rad/s

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Wheel rotation state: FL, FR, RL, RR
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]

        # --- Publishers & Broadcasters ---
        qos = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos)
        self.odom_pub  = self.create_publisher(Odometry, 'odom', qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Subscriber & Timer ---
        self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            qos)
        self.create_timer(1.0 / rate_hz, self.update)

    def cmd_vel_callback(self, msg: Twist):
        # Linear velocity (m/s)
        self.v = msg.linear.x
        # Normalize steering [-1..1], scale to max_steer_angle
        raw = msg.angular.z
        raw = max(min(raw, 1.0), -1.0)
        self.delta = raw * self.max_steer_angle

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # 1) Ramp steering toward target
        steer_err = self.delta - self.current_delta
        max_step = self.max_steer_rate * dt
        if steer_err > max_step:
            self.current_delta += max_step
        elif steer_err < -max_step:
            self.current_delta -= max_step
        else:
            self.current_delta = self.delta

        # 2) Compute yaw-rate (bicycle model)
        omega = 0.0
        if abs(self.v) > 1e-4:
            omega = self.v * math.tan(self.current_delta) / self.wheelbase

        # 3) Integrate pose
        self.x  += self.v * math.cos(self.th) * dt
        self.y  += self.v * math.sin(self.th) * dt
        self.th += omega * dt

        # 4) Publish TF (odom → base_link)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id    = 'odom'
        t.child_frame_id     = 'base_link'
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        q = self.get_quaternion_from_yaw(self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # 5) Publish Odometry message
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x    = self.v
        odom.twist.twist.angular.z   = omega
        self.odom_pub.publish(odom)

        # 6) Integrate wheel rotation (rad)
        omega_wheel = 0.0
        if abs(self.v) > 1e-4:
            omega_wheel = self.v / self.rear_radius
        for i in range(len(self.wheel_pos)):
            self.wheel_pos[i] += omega_wheel * dt

        # 7) Publish JointState for all joints
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = [
            'front_left_hinge_joint', 'front_left_joint',
            'front_right_hinge_joint','front_right_joint',
            'rear_left_joint','rear_right_joint'
        ]
        js.position = [
            self.current_delta,    self.wheel_pos[0],
            self.current_delta,    self.wheel_pos[1],
            self.wheel_pos[2],     self.wheel_pos[3]
        ]
        js.velocity = [
            0.0, omega_wheel,
            0.0, omega_wheel,
            omega_wheel, omega_wheel
        ]
        self.joint_pub.publish(js)

    @staticmethod
    def get_quaternion_from_yaw(yaw: float):
        qz = np.sin(yaw * 0.5)
        qw = np.cos(yaw * 0.5)
        return (0.0, 0.0, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckonWithJoints()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
