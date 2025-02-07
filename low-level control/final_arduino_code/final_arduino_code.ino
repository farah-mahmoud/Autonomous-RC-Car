#include <Servo.h>

Servo steeringServo;
Servo ESC;

int steeringAngle = 90;
float throttleValue = 1500;

void setup() {
  Serial.begin(115200);
  steeringServo.attach(9);
  ESC.attach(10, 1000, 2000);

  ESC.writeMicroseconds(1500);
  delay(3000);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read input
    int commaIndex = input.indexOf(',');
    
    if (commaIndex != -1) {
      int linear_x = input.substring(0, commaIndex).toInt();
      int angular_z = input.substring(commaIndex + 1).toInt();

      // Convert linear velocity to throttle value
      throttleValue = map(linear_x, -100, 100, 1000, 2000);

      // Convert angular velocity to steering angle
      steeringAngle = map(angular_z, -100, 100, 50, 130);

      steeringServo.write(steeringAngle);
      ESC.writeMicroseconds(throttleValue);
      
      // Debugging
      Serial.print("Received: ");
      Serial.println(input);
      Serial.print("Throttle: ");
      Serial.println(throttleValue);
      Serial.print("Steering: ");
      Serial.println(steeringAngle);
    }
  }
}