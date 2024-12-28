# Example Arduino Code
```cpp
#include <Servo.h>
#include <Wire.h>
#include "MPU6050.h"

Servo steeringServo;
Servo motorESC;

MPU6050 mpu;

void setup() {
  // Initialize servo and ESC
  steeringServo.attach(9); // Pin for servo
  motorESC.attach(10);     // Pin for ESC

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Serial communication for commands
  Serial.begin(9600);
  Serial.println("System ready!");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'w') {
      motorESC.write(60); // Forward
    } else if (command == 's') {
      motorESC.write(90); // Stop
    } else if (command == 'x') {
      motorESC.write(120); // Reverse
    } else if (command == 'a') {
      steeringServo.write(45); // Turn left
    } else if (command == 'd') {
      steeringServo.write(135); // Turn right
    } else if (command == 'c') {
      steeringServo.write(90); // Center steering
    }
  }

  // Optional: Read MPU6050 data for orientation
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" Y: "); Serial.print(ay);
  Serial.print(" Z: "); Serial.println(az);
  delay(100);
}

```
# Example Rpi node:
```python
import serial
import time

# Set up serial communication
arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=1)

def send_command(command):
    arduino.write(command.encode())
    time.sleep(0.1)

# Simple teleoperation
print("Enter commands: w (forward), s (stop), x (reverse), a (left), d (right), c (center)")
while True:
    user_input = input("Command: ")
    if user_input in ['w', 's', 'x', 'a', 'd', 'c']:
        send_command(user_input)
    else:
        print("Invalid command.")
```

# Read from serial monitor  
create this node:

```python
import serial

# Replace '/dev/ttyACM0' with your Arduino port
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)

while True:
    data = arduino.readline().decode('utf-8').strip()
    if data:
        print(data)
```
