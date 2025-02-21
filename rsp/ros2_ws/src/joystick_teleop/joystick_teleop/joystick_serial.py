import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import serial

class JoystickSerial(Node):
    def __init__(self):
        super().__init__('joystick_serial')

        # Subscribe to joystick topic
        self.subscription = self.create_subscription(
            Joy,
            '/joy',  
            self.joy_callback,
            10
        )
        self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)  # Adjust if needed

        self.linear_scale = 100  # Scale joystick input to -100 to 100
        self.angular_scale = 100  # Scale joystick input to -100 to 100

    def joy_callback(self, msg):
        # Left joystick: Y-axis (forward/back), X-axis (left/right)
        linear_x = int(msg.axes[1] * self.linear_scale)
        angular_z = int(msg.axes[0] * self.angular_scale)

        # Format and send command over serial
        command = f"{linear_x},{angular_z}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
