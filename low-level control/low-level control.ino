#include "Wire.h"              // Library for I2C communication
#include <MPU6050_light.h>     // MPU6050 sensor library for accelerometer and gyroscope
#include "ESC.h"               // Library to control ESCs (Electronic Speed Controllers)

/* Pin Definitions */
#define LED_PIN (13)            // Optional LED pin for debugging purposes
#define SPEED_MIN (1080)        // Minimum ESC speed signal (in microseconds)
#define SPEED_NEUTRAL (1540)    // Neutral ESC speed signal (car stationary)
#define SPEED_MAX (2000)        // Maximum ESC speed signal (full speed forward)
#define STEERING_MIN (1100)     // Minimum steering signal (full left)
#define STEERING_MAX (1900)     // Maximum steering signal (full right)

/* MPU6050 Object and Timer */
MPU6050 mpu(Wire);              // MPU6050 object to interface with the IMU sensor
unsigned long timer = 0;        // Timer variable for periodic updates

/* ESC and Control Variables */
float true_angle = 0.0;         // Variable to store the desired yaw angle (Z-axis) for correction
ESC steeringESC(11, STEERING_MIN, STEERING_MAX, 1520);  // ESC object for steering (Pin 11)
ESC speedESC(9, SPEED_MIN, SPEED_MAX, 1500);           // ESC object for speed control (Pin 9)

float speedInput = 0.0;         // Input for speed from serial communication
float steeringInput = 0.0;      // Input for steering from serial communication
float controlled_speed = 0.0;   // Gradual control for smooth acceleration
float controlled_steer = 0.0;   // Variable to control steering correction
int speed_flag = 0;             // Flag to ensure gradual acceleration happens only once

/* 
 * Setup Function: 
 * --------------------
 * - Initializes serial communication at 57600 baud for receiving control commands.
 * - Initializes I2C communication and calibrates the MPU6050 sensor.
 * - Arms the ESCs for speed and steering control (safety feature before motion).
 * - Prints debug messages to confirm successful initialization and offsets.
 */
void setup() {
  Serial.begin(57600);          // Initialize serial communication at 57600 baud rate
  Wire.begin();                 // Start I2C communication (used for MPU6050)

  // Initialize and calibrate the MPU6050 sensor
  byte status = mpu.begin();
  Serial.print("MPU6050 status: ");  // Print the initialization status
  Serial.println(status);
  while (status != 0) { }       // Stop everything if MPU6050 initialization fails

  Serial.println("Calculating MPU offsets...");
  delay(1000);                  // Wait before starting the calibration
  mpu.calcOffsets();            // Calculate gyro and accelerometer offsets
  Serial.println("MPU offsets calculated!");

  // Arm the ESCs (required to start the ESCs safely)
  speedESC.arm();
  steeringESC.arm();
}

/* 
 * Loop Function: 
 * --------------------
 * - Reads speed and steering values from Serial input (format: "steering speed").
 * - Updates ESC signals based on mapped values for speed and steering control.
 * - Gradually increases speed during acceleration for smooth motion.
 * - Auto-corrects the heading (yaw) if no steering input is provided, ensuring the car drives straight.
 * - Maintains a 25ms delay for periodic updates, ensuring stable control.
 */
void loop() {
  // Read and process serial commands every 25ms
  if ((millis() - timer) > 25) {
    if (Serial.available() > 0) {
      // Read serial command in the format: "steering speed"
      String message = Serial.readStringUntil('\n');   // Read full line from Serial
      int delimiterPosition = message.indexOf(' ');    // Find the space separating steering and speed values

      if (delimiterPosition != -1) {                  // Check if space delimiter exists
        // Extract steering input from the message
        String steeringString = message.substring(0, delimiterPosition);
        String speedString = message.substring(delimiterPosition + 1);

        // Convert inputs to float values
        steeringInput = steeringString.toFloat();     
        controlled_speed = speedString.toFloat();     
      }

      // Map steering and speed values to ESC signal ranges
      int steeringSignal = map(int(steeringInput * 100), -70, 70, STEERING_MIN, STEERING_MAX);
      int speedSignal = map(int(controlled_speed * 100), 0, 850, SPEED_NEUTRAL, SPEED_MAX);

      // Send signals to ESCs
      speedESC.speed(speedSignal);       // Control the speed ESC
      steeringESC.speed(steeringSignal); // Control the steering ESC

      // Smooth acceleration logic for controlled speed
      if ((controlled_speed > 0) && (speed_flag < 1)) {
        mpu.update();  // Update IMU readings
        // Gradually increase speed until acceleration threshold is reached
        while (((mpu.getAccY() * 9.81) < 0.25) && (controlled_speed < 0.88)) {
          controlled_speed += 0.007;   // Increment speed gradually
          speedSignal = map(int(controlled_speed * 100), 0, 850, SPEED_NEUTRAL, SPEED_MAX);
          speedESC.speed(speedSignal); // Update ESC with new speed
          delay(10);                   // Small delay for smooth acceleration
          mpu.update();                // Update IMU readings again
        }
        speed_flag++;                  // Set flag to avoid repeating this process
      }

      // Correct steering if no active steering input (auto-correction mode)
      if (steeringInput == 0) {
        mpu.update();  // Update IMU readings
        // Check if current angle deviates from desired true_angle by more than 0.5 degrees
        if (abs(mpu.getAngleZ() - true_angle) > 0.5) {
          if (mpu.getAngleZ() < true_angle) {  // If car turns left (negative deviation)
            steeringInput += 0.02;            // Adjust steering to the right
          } else {                            // If car turns right (positive deviation)
            steeringInput -= 0.02;            // Adjust steering to the left
          }
          // Map corrected steering input to ESC signal
          steeringSignal = map(int(steeringInput * 100), -70, 70, STEERING_MIN, STEERING_MAX);
          steeringESC.speed(steeringSignal); // Apply steering correction
        } else {
          true_angle = mpu.getAngleZ();  // Update true_angle if car is stable
        }
      }
    }
    timer = millis();  // Reset timer to maintain 25ms loop timing
  }
}

/*
 * Expected Output:
 * --------------------
 * 1. The car moves forward or backward depending on the "speed" value sent via Serial.
 * 2. The car steers left or right based on the "steering" value sent via Serial.
 * 3. If no steering input is provided, the car automatically corrects its heading using MPU6050 data.
 * 4. Debug messages appear in the Serial Monitor, including MPU6050 initialization and status updates.
 *
 * Connections to Arduino UNO:
 * --------------------
 * - Pin 9: ESC signal for controlling the car's speed.
 * - Pin 11: ESC signal for controlling the car's steering.
 * - I2C Pins (A4, A5): Connected to the MPU6050 sensor.
 * - 5V and GND: Power supply for MPU6050 and ESCs.
 *
 * Role of Raspberry Pi:
 * --------------------
 * - The Raspberry Pi acts as the main controller, sending speed and steering commands to the Arduino via Serial communication.
 * - The Raspberry Pi can run advanced algorithms like path planning, object detection, or remote control scripts.
 */
