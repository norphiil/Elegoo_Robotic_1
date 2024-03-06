/**
 * This sketch tests the movement of the robot in all 8 directions,
 * an example error direction (22), and stops the robot at the end.
 * It moves in each direction for 1 second.
 */

#include "devices.h"
#include <avr/wdt.h>

Motor motor;

void setup()
{
  Serial.begin(9600);

  motor.init();
  delay(2000);

  motor.move(FORWARDS, 100);
  delay(1000);
  motor.move(BACKWARDS, 100);
  delay(1000);
  motor.move(RIGHT, 100);
  delay(1000);
  motor.move(LEFT, 100);
  delay(1000);
  motor.move(FORWARDS_RIGHT, 100);
  delay(1000);
  motor.move(FORWARDS_LEFT, 100);
  delay(1000);
  motor.move(BACKWARDS_RIGHT, 100);
  delay(1000);
  motor.move(BACKWARDS_LEFT, 100);
  delay(1000);
  // Error direction
  motor.move(22, 100);
  delay(1000);
  motor.stop();
}

void loop()
{
  // put your main code here, to run repeatedly:
}
