import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist


class ArduinoNode(Node):
    def __init__(self):  # Corrected the constructor method
        super().__init__('arduino_node')
        self.serial_port = serial.Serial('/dev/ttyACM0', 57600)  # Adjust the port and baud rate
        self.subscription = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.callback,
            10)

    def callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Format the data as a string to send to the Arduino
        data_to_send = f"{linear_x} {angular_z}"
        self.get_logger().info(f"Sending data: {data_to_send.strip()}")  # Log the data being sent

        # Send the message to the Arduino if serial port is initialized
        if self.serial_port:
            self.serial_port.write(data_to_send.encode())
        else:
            self.get_logger().warn("Serial port is not initialized. Cannot send data to Arduino.")


def main(args=None):
    rclpy.init(args=args)
    arduino_node = ArduinoNode()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':  # Corrected this line
    main()

