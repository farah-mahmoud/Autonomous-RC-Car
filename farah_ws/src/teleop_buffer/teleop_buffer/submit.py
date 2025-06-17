#!/usr/bin/env python3
# Licensed under BSD License 2.0

import sys
import threading
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


msg = """
This node latches the last keypress as a continuous velocity command.
Press one of:

   u    i    o
   j    k    l
   m    ,    .

to drive:
  i = forward
  , = backward
  j = turn left
  l = turn right

Holonomic (strafing) with Shift:
   U    I    O
   J    K    L
   M    <    >

t : up (+z)      b : down (-z)

k : EMERGENCY STOP (zeros velocities)

q/z : increase/decrease both speeds by 10%
w/x : increase/decrease linear speed only
e/c : increase/decrease angular speed only

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1.0),
    'x': (0.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, 0.9),
}


def getKey(settings):
    """Read one keypress from stdin (raw mode)."""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    """Save current terminal settings (for Linux/mac)."""
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    """Restore saved terminal settings."""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    settings = saveTerminalSettings()

    rclpy.init()
    node = Node('teleop_twist_keyboard_latch')

    # Publisher: bare Twist on /cmd_vel
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Latched state
    current_x = 0.0
    current_th = 0.0
    speed = 0.5
    turn = 1.0

    twist_msg = Twist()

    # Timer callback: publish at 10 Hz
    def timer_callback():
        twist_msg.linear.x = current_x * speed
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = current_th * turn
        pub.publish(twist_msg)

    node.create_timer(0.1, timer_callback)

    # Thread to read keys and update latched velocities
    def key_loop():
        nonlocal current_x, current_th, speed, turn
        try:
            print(msg)
            print(f"Latched velocities: x={current_x}, θ={current_th}")
            while True:
                key = getKey(settings)
                if key in moveBindings:
                    current_x, _, _, current_th = moveBindings[key]
                    print(f"Latched to: x={current_x}, θ={current_th}")
                elif key in speedBindings:
                    speed *= speedBindings[key][0]
                    turn *= speedBindings[key][1]
                    print(f"Speed scales → linear={speed:.2f}, angular={turn:.2f}")
                elif key == 'k':
                    current_x = 0.0
                    current_th = 0.0
                    print("EMERGENCY STOP: velocities zeroed")
                elif key == '\x03':  # CTRL-C
                    break
                # otherwise ignore: keep last command latched
        finally:
            restoreTerminalSettings(settings)
            rclpy.shutdown()

    thread = threading.Thread(target=key_loop, daemon=True)
    thread.start()

    # Spin ROS in main thread
    rclpy.spin(node)
    thread.join()


if __name__ == '__main__':
    main()

