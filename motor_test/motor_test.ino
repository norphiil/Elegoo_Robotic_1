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

  // motor.move(FORWARDS, 100);
  // delay(1000);
  // motor.move(BACKWARDS, 100);
  // delay(1000);
  // motor.move(RIGHT, 100);
  // delay(1000);
  // motor.move(LEFT, 100);
  // delay(1000);
  // motor.move(FORWARDS_RIGHT, 100);
  // delay(1000);
  // motor.move(FORWARDS_LEFT, 100);
  // delay(1000);
  // motor.move(BACKWARDS_RIGHT, 100);
  // delay(1000);
  // motor.move(BACKWARDS_LEFT, 100);
  // delay(1000);
  // // Error direction
  // motor.move(22, 100);
  // delay(1000);
  // motor.stop();

  Pos pos1;
  pos1.init(1, 0);
  Pos pos2;
  pos2.init(0, 1);
  Pos pos3;
  pos3.init(1, 1);
  Pos pos4;
  pos4.init(0, 0);

  Pos path_list[4] = {pos1, pos2, pos3, pos4};

  Path path;
  path.init(path_list);
  path.run(motor, 100);
}

void loop()
{
  // put your main code here, to run repeatedly:
}
