/**
 * This sketch tests the movement of the robot in all 8 directions,
 * an example error direction (22), and stops the robot at the end.
 * It moves in each direction for 1 second.
 */
#include <Arduino.h>
#include "device_motor.h"
#include <avr/wdt.h>

Motor motor;
float initialYaw = 0;

unsigned long lastTime = 0;

void setup()
{
  Serial.begin(9600);
  Serial.println(" Starting...");
  motor.init();
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
  // float roll, pitch, currentYaw;
  // motor.getGyroAccel()->getRotation(&roll, &pitch, &currentYaw);
  // initialYaw = currentYaw;

  // float roll, pitch, currentYaw;
  // motor.getGyroAccel()->getRotation(&roll, &pitch, &currentYaw);
  // Serial.print("Yaw: ");
  // Serial.println(currentYaw);
  // motor.turn(90, 35);
  // motor.stop();
  // motor.turn(90, 35);
  // motor.stop();
  // motor.getGyroAccel()->getRotation(&roll, &pitch, &currentYaw);
  // Serial.print("After: ");
  // Serial.println(currentYaw);
}

void loop()
{
  float roll, pitch, currentYaw;
  motor.getGyroAccel()->getRotation(&roll, &pitch, &currentYaw);
  motor.straightLine(50, currentYaw);
  delay(10);
  motor.servo.setAngle(0);
  uint16_t left_distance = motor.ultrasonic.get_distance();
  motor.servo.setAngle(180);
  uint16_t right_distance = motor.ultrasonic.get_distance();
  motor.servo.setAngle(90);
  motor.getGyroAccel()->getRotation(&roll, &pitch, &currentYaw);
  if (abs(left_distance - right_distance) < 20)
  {
    motor.turn(180, 35);
    motor.stop();
  }
  else if (left_distance - right_distance > 0)
  {
    motor.turn(270, 35);
    motor.stop();
  }
  else if (right_distance - left_distance > 0)
  {
    motor.turn(90, 35);
    motor.stop();
  }
}
