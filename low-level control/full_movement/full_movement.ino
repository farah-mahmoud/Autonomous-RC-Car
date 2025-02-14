#include <Servo.h>

Servo steeringServo;
Servo ESC;

int steeringAngle = 90;
int throttleValue = 1500;
bool reverseReady = false; // Flag to track reverse readiness

void setup() {
  Serial.begin(115200);
  steeringServo.attach(9);
  ESC.attach(10, 1000, 2000);

  ESC.writeMicroseconds(1500);  // Neutral position
  delay(3000);
}

// Function to prepare ESC for reverse movement
void prepareReverse() {
  ESC.writeMicroseconds(1500);  // Return to Neutral
  delay(500);  // Small delay for ESC to register full reverse
  ESC.writeMicroseconds(1250);  // Full reverse throttle
  delay(500);  // Allow ESC to reset
  reverseReady = true;  // Mark reverse as ready
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read input
    int commaIndex = input.indexOf(',');

    if (commaIndex != -1) {
      int linear_x = input.substring(0, commaIndex).toInt();
      int angular_z = input.substring(commaIndex + 1).toInt();

      // Determine throttle value based on forward or backward movement
      if (linear_x > 0) {
        throttleValue = map(linear_x, 0, 100, 1500, 2000); // Forward
        reverseReady = false; // Reset reverse readiness when moving forward
      } else if (linear_x < 0) {
        if (!reverseReady) {
          prepareReverse();  // Ensure reverse is ready before applying throttle
        }
        throttleValue = map(linear_x, -100, 0, 1000, 1500); // Backward
      } else {
        throttleValue = 1500; // Stop
      }

      // Convert angular velocity to steering angle
      steeringAngle = map(angular_z, -100, 100, 50, 130);

      // Send commands to servo and ESC
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
