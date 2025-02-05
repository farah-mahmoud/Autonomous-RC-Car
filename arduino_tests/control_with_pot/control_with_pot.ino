#include <Servo.h>

Servo steeringServo;  // Servo for steering
Servo ESC;           // ESC for throttle control

int steeringPot = A0;  // Potentiometer for steering
int throttlePot = A1;  // Potentiometer for throttle

void setup() {
  Serial.begin(9600);

  steeringServo.attach(9);        // Attach steering servo to pin 9
  ESC.attach(10, 1500, 1900);      // Attach ESC to pin 10 (Throttle control)
  
  ESC.writeMicroseconds(1500);  // Set ESC to neutral (stop position)
  delay(2000);  // Wait for ESC initialization
}

void loop() {
  // **Read Potentiometer Values**
  int steeringVal = analogRead(steeringPot);  
  int throttleVal = analogRead(throttlePot);  

  // **Steering Control (Servo: 50° - 130°)**
  steeringVal = map(steeringVal, 0, 1023, 50, 130);
  steeringServo.write(steeringVal);

  // **Throttle Control (ESC: 1500µs - 1900µs)**
  throttleVal = map(throttleVal, 0, 1023, 1500, 1900);
  throttleVal = (throttleVal / 100) * 100;  // Round to nearest 100
  ESC.writeMicroseconds(throttleVal);

  // **Debugging Output**
  Serial.print("Steering: ");
  Serial.print(steeringVal);
  Serial.print("° | Throttle: ");
  Serial.println(throttleVal);

  delay(100);  // Small delay for stability
}
