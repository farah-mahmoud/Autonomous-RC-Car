import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.serial_port = serial.Serial('/dev/ttyACM0', 57600)  # Adjust the port and baud rate
        self.subscription = self.create_subscription(
            TwistStamped,
            '/asc/reference',
            self.callback,
            10)

    def callback(self, msg):
        if msg.twist:
            linear_x = msg.twist.linear.x
            angular_z = msg.twist.angular.z
            
            # Format the data as a string to send to the Arduino
            data_to_send = f"Linear X: {linear_x}, Angular Z: {angular_z}"
            self.get_logger().info("Sending data: {}".format(data_to_send.strip()))  # Log the data being sent
            
            # Send the message to the Arduino if serial port is initialized
            if self.serial_port:
                self.serial_port.write(data_to_send.encode())  
            else:
                self.get_logger().warn("Serial port is not initialized. Cannot send data to Arduino.")
        else:
            self.get_logger().warn("Received message does not contain Twist data.")


def main(args=None):
    rclpy.init(args=args)
    arduino_node = ArduinoNode()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
