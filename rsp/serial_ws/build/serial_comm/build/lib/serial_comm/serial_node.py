import rclpy
from rclpy.node import Node
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.timer = self.create_timer(1.0, self.send_and_receive)

    def send_and_receive(self):
        # Send message to Arduino
        self.serial_port.write(b"Hello Arduino!\n")
        self.get_logger().info("Sent: Hello Arduino!")

        # Read response from Arduino
        if self.serial_port.in_waiting > 0:
            response = self.serial_port.readline().decode('utf-8').strip()
            self.get_logger().info(f"Received: {response}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
