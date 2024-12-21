
# Setting Up and Testing Serial Communication Between Arduino and Raspberry Pi Using SSH and ROS 2 Foxy

## 1. Connecting the Arduino to Raspberry Pi
1. Connect the Arduino to the Raspberry Pi using a USB cable (the same cable used for uploading code to the Arduino).

2. Identify the serial port on the Raspberry Pi:
   ```bash
   ls /dev/tty*
   ```
   Look for a device like `/dev/ttyUSB0` or `/dev/ttyACM0`.

3. Ensure proper permissions for the serial port:
   ```bash
   sudo usermod -a -G dialout $USER
   ```
   Log out and log back in for the changes to take effect.

---

## 2. Writing the Arduino Code
1. Open the Arduino IDE and write a simple echo program:
   ```cpp
   void setup() {
       Serial.begin(9600); // Start serial communication at 9600 baud rate
   }

   void loop() {
       if (Serial.available()) {
           String message = Serial.readString();
           Serial.print("Echo: ");
           Serial.println(message); // Echo back the received message
       }
   }
   ```

2. Upload the code to the Arduino.

---

## 3. Setting Up ROS 2 Foxy on Raspberry Pi
1. Ensure ROS 2 Foxy is installed on the Raspberry Pi. Follow the [official installation guide](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) if needed.

2. Create a new workspace for your project:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   ```

3. Initialize the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

---

## 4. Writing the ROS 2 Node for Serial Communication
1. Navigate to the `src` directory of your workspace:
   ```bash
   cd ~/ros2_ws/src
   ```

2. Create a new ROS 2 package for serial communication:
   ```bash
   ros2 pkg create --build-type ament_python serial_comm --dependencies rclpy
   ```

3. Navigate to the package directory and edit the `serial_comm/serial_comm/serial_node.py` file:
   ```python
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
   ```

4. Update the `setup.py` file in your package to include the `serial_node.py` script.

---

## 5. Building and Running the ROS 2 Node
1. Return to your workspace root and build the package:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

2. Run the ROS 2 node:
   ```bash
   ros2 run serial_comm serial_node
   ```

3. Monitor the output:
   - Sent messages: "Hello Arduino!"
   - Received messages: "Echo: Hello Arduino!"

---

## 6. Transferring Files to Raspberry Pi (Optional)
If developing on a separate machine, transfer the `serial_comm` package to the Raspberry Pi using `scp`:
```bash
scp -r /path/to/serial_comm pi_user@pi_ip:/home/pi/ros2_ws/src/
```

Then rebuild the workspace on the Raspberry Pi as described in step 5.

---

## 7. Debugging Tips
- Ensure the Arduino code works independently by testing it in the Arduino Serial Monitor.
- Verify the correct serial port (`/dev/ttyUSB0`, `/dev/ttyACM0`, etc.) using `ls /dev/tty*`.
- Check permissions for the serial port.
