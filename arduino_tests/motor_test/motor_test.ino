#include <Servo.h> // Include the Servo library
#include <ESC.h>   // Include the ESC library

// Define constants
#define SPEED_MIN (1080)                                  // Set the Minimum Speed in microseconds
#define SPEED_NEUTRAL (1540)                              // Set the neutral
#define SPEED_MAX (2000)                                  // Set the Maximum Speed in microseconds

// Initialize ESC objects
ESC myESC (9, SPEED_MIN, SPEED_MAX, 1500);              // ESC for Speed
// ESC myESC2;  // ESC for speed control

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Attach ESCs to the respective pins
  myESC.attach(9);   // Steering ESC connected to pin 9
  // myESC2.attach(10); // Speed ESC connected to pin 10

  // Set initial speeds to neutral
  myESC.setSpeed(SPEED_MIN);
  // myESC2.setSpeed(SPEED_NEUTRAL);

  Serial.println("ESCs initialized and set to neutral.");
}

void loop() {
  for (int inc = -500; inc <= 2000; inc += 100){
    myESC.setSpeed(inc);
    delay(2000);
  }
  /*
    myESC.setSpeed(500); // Set speed for the speed ESC
    delay(2000);

    myESC.setSpeed(600); // Set speed for the speed ESC
    delay(2000);

    myESC.setSpeed(1200); // Set speed for the speed ESC
    delay(2000);
    */
/*
  // Example: Gradually increase speed for the first ESC (steering)
  Serial.println("Increasing speed...");
  for (int i = SPEED_MIN; i <= SPEED_MAX; i += 10) {
    myESC.setSpeed(i); // Set speed for the steering ESC
    delay(10);         // Small delay for smooth change
  }

  // Reset to neutral speed
  // delay(1000); // Pause for observation
  // myESC.setSpeed(SPEED_NEUTRAL);

  // Example: Gradually decrease speed for the second ESC (speed control)
  Serial.println("Decreasing speed...");
  for (int i = SPEED_MAX; i >= SPEED_MIN; i -= 10) {
    myESC.setSpeed(i); // Set speed for the speed ESC
    delay(10);          // Small delay for smooth change
  }

  // Reset to neutral speed
  // myESC2.setSpeed(SPEED_NEUTRAL);
  // delay(1000); // Pause for observation
*/
}