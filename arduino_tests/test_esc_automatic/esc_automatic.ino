/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

#define MIN_SPEED 1500
#define MAX_SPEED 1700

Servo esc;  // create Servo object to control a servo

// int potpin = A0;  // analog pin used to connect the potentiometer
// int val;    // variable to read the value from the analog pin

void setup() {
  Serial.begin(9600);
  esc.attach(9);  // attaches the servo on pin 9 to the Servo object
  Serial.println("Init ...");
  Serial.println(1500);
  esc.writeMicroseconds(1500);
  delay(3000);
  Serial.println("Armed ...");
}

void loop() {
  // val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  // val = map(val, 0, 1023, 0, 180);     // scale it for use with the servo (value between 0 and 180)
  Serial.println("Speed Increasing ...");
  for (int v = MIN_SPEED; v < MAX_SPEED; v += 10){
    esc.writeMicroseconds(v);
    Serial.println(v);
    delay(20);
  }
  Serial.println("Speed Decreasing ...");
  for (int v = MAX_SPEED; v > MIN_SPEED; v -= 10){
    esc.writeMicroseconds(v);
    Serial.println(v);
    delay(20);
  }
  //delay(2000);
  /*
  esc.writeMicroseconds(2500);
  Serial.println("2500");
  delay(2000);                           // waits for the servo to get there
  esc.writeMicroseconds(2000);
  Serial.println("2000");
  delay(2000);                           // waits for the servo to get there
  esc.writeMicroseconds(3000);
  Serial.println("3000");
  delay(2000);                           // waits for the servo to get there
*/
}