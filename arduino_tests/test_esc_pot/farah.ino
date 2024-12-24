/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

#include <Servo.h>

Servo ESC;  // create Servo object to control a servo

int potpin = A0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin

void setup() {
  Serial.begin(9600);
  ESC.attach(9, 1500, 1900);  // attaches the servo on pin 9 to the Servo object
  ESC.writeMicroseconds(1500);
  delay(2000);
}

void loop() {
  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 1500, 1900);     // scale it for use with the servo (value between 0 and 180)
  val = val/100;
  val = val*100;
  ESC.writeMicroseconds(val);                  // sets the servo position according to the scaled value
  Serial.println(val);
  delay(100);
}
