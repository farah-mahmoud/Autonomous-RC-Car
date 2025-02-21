import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')

        # Initialize the serial connection for GPIO UART
        try:
            self.serial_port = serial.Serial('/dev/ttyAMA0',57600, timeout=1)  # Adjust baud rate if needed
            time.sleep(2)  # Allow the connection to stabilize
            self.get_logger().info("Serial connection established via GPIO UART.")
        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f"Failed to connect to Arduino via GPIO UART: {e}")

        # Subscribe to the Twist topic
        self.subscription = self.create_subscription(
            Twist,
            '/diff_cont/cmd_vel_unstamped',
            self.callback,
            10
        )

    def callback(self, msg):
        # Extract linear and angular velocity from the Twist message
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Format the data as a string to send to the Arduino
        data_to_send = f"{linear_x} {angular_z}\n"  # Add newline for proper Arduino parsing
        self.get_logger().info(f"Sending data: {data_to_send.strip()}")  # Log the data being sent

        # Send the data to the Arduino via GPIO UART
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(data_to_send.encode())
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send data to Arduino: {e}")
        else:
            self.get_logger().warn("Serial port is not open. Cannot send data to Arduino.")

    def destroy_node(self):
        # Close the serial connection when the node is destroyed
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial connection closed.")
        super().destroy_node()  # Call the parent class's destroy_node method


def main(args=None):
    rclpy.init(args=args)
    arduino_node = ArduinoNode()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


