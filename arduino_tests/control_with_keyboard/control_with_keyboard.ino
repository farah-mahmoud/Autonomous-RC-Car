#include <Servo.h>

Servo steeringServo;
Servo ESC;

int steeringAngle = 90;
float throttleValue = 1500;
bool firstForwardPress = true;   // Flag to track first forward press
bool firstBackwardPress = true;  // Flag to track first forward press

void setup() {
  Serial.begin(9600);
  steeringServo.attach(9);
  ESC.attach(10, 1000, 2000);

  ESC.writeMicroseconds(1500);
  delay(3000);

  Serial.println("Use the following keys to control:");
  Serial.println("W: Increase Throttle (Forward)");
  Serial.println("S: Decrease Throttle (Slow Down)");
  Serial.println("B: Move Backward");
  Serial.println("N: Move Backward (Slow Down)");
  Serial.println("A: Steer Left");
  Serial.println("D: Steer Right");
  Serial.println("X: Center Steering");
  Serial.println("Space: Stop Throttle");
}

void loop() {
  if (Serial.available() > 0) {
    char key = Serial.read();

    // Throttle Control
    if (key == 'w' || key == 'W') {
      
        throttleValue += 1;
      
      throttleValue = constrain(throttleValue, 1570, 1700);
    }
     else if (key == 's' || key == 'S') {
      throttleValue -= 1;
      throttleValue = constrain(throttleValue, 1570, 1700);
    }

    // Backward
    else if (key == 'b' || key == 'B') {
      throttleValue -= 50;
      /*
      if (firstBackwardPress) {
        throttleValue = 1500;
        firstBackwardPress = false;
      } else {
        throttleValue -= 50;
      }
      */
      throttleValue = constrain(throttleValue, 1000, 1400);

    } else if (key == 'n' || key == 'N') {
      throttleValue += 50;
      throttleValue = constrain(throttleValue, 1000, 1400);
    }

    else if (key == ' ') {
      throttleValue = 1500;
      firstForwardPress = true;   // Reset flag when stopping
      firstBackwardPress = true;  // Reset backward flag
    }

    // Steering Control
    if (key == 'a' || key == 'A') {
      steeringAngle -= 10;
    } else if (key == 'd' || key == 'D') {
      steeringAngle += 10;
    } else if (key == 'x' || key == 'X') {
      steeringAngle = 90;
    }

    steeringAngle = constrain(steeringAngle, 50, 130);

    // Apply changes
    steeringServo.write(steeringAngle);
    ESC.writeMicroseconds(throttleValue);

    // Debugging Output
    Serial.print("Steering: ");
    Serial.print(steeringAngle);
    Serial.print("° | Throttle: ");
    Serial.println(throttleValue);
  }

  delay(100);
}
