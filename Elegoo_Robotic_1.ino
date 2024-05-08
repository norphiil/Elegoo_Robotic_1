/**
 * This sketch tests the movement of the robot in all 8 directions,
 * an example error direction (22), and stops the robot at the end.
 * It moves in each direction for 1 second.
 */
#include <Arduino.h>
#include "device_motor.h"
#include <avr/wdt.h>

Motor motor;

unsigned long lastTime = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println(" Starting...");
  motor.init();
  delay(2000);
  lastTime = millis();

  // float roll, pitch, yaw;
  // motor.turn(90 * 1, 100);
  // delay(2000);
  // motor.turn(90 * 2, 100);
  // delay(2000);
  // motor.turn(90 * 3, 100);
  // delay(2000);
  // motor.turn(90 * 4, 100);
  // delay(2000);
  float roll, pitch, currentYaw;
  motor.getGyroAccel()->getRotation(&roll, &pitch, &currentYaw);
  delay(1000);
  motor.getGyroAccel()->getRotation(&roll, &pitch, &currentYaw);
  delay(100);
  motor.straightLine(50, currentYaw);
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
