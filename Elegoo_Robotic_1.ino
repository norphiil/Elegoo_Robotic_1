/**
 * This sketch tests the movement of the robot in all 8 directions,
 * an example error direction (22), and stops the robot at the end.
 * It moves in each direction for 1 second.
 */

#include "devices.h"
#include <avr/wdt.h>

Motor motor;
GyroAccel gyroAccel;
Ultrasonic ultrasonic;
Servo servo;

unsigned long lastTime = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println(" Starting...");
  motor.init();
  // ultrasonic.init();
  // servo.init();
  delay(2000);
  lastTime = millis();
  // motor.obstacle_avoidance();

  // float roll, pitch, yaw;
  // motor.turn(90 * 1, 100);
  // delay(2000);
  // motor.turn(90 * 2, 100);
  // delay(2000);
  // motor.turn(90 * 3, 100);
  // delay(2000);
  // motor.turn(90 * 4, 100);
  // delay(2000);

  motor.straightLine(50, 0);
  delay(10);
}

void loop()
{
  // unsigned long currentTime = millis();
  // float roll, pitch, yaw;
  // // motor.gyroaccel.getRotation(&roll, &pitch, &yaw);
  // motor.getGyroAccel()->getRotation(&roll, &pitch, &yaw);
  // if (currentTime - lastTime > 1000)
  // {
  //   Serial.print("Yaw: ");
  //   Serial.println(yaw);
  //   lastTime = currentTime;
  // }
}
