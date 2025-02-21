import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Adjust the port and baud rate
        self.subscription = self.create_subscription(
            String,
            'input_numbers',
            self.callback,
            10)

    def callback(self, msg):
        num_msg = msg.data
        try:
            # Parse the received message to extract two numbers
            num1, num2 = map(int, num_msg.split())
            
            # Check if both numbers are within the specified ranges
            if 0 <= num1 <= 99 and -45 <= num2 <= 45:
                # Format the numbers as a string to send to the Arduino
                data_to_send = f"{num1} {num2}\n"
                print("Sending data:", data_to_send)  # Print the data being sent
                self.serial_port.write(data_to_send.encode())  # Sending the message to Arduino
            else:
                self.get_logger().warn("First number must be in the range 0 to 99 and second number must be in the range -45 to 45.")
        except ValueError:
            self.get_logger().warn("Invalid input. Please enter two valid numbers separated by a space.")

def main(args=None):
    rclpy.init(args=args)
    arduino_node = ArduinoNode()
    rclpy.spin(arduino_node)
    arduino_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
