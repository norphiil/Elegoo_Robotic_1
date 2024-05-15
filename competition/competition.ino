#include <Arduino.h>
#include "device_motor.h"
#include <avr/wdt.h>

// Maze maze;
Motor motor;
float initialYaw = 0;

unsigned long lastTime = 0;

void setup() {
  // maze.init(6, 6);


  const int SPEED = 100;
  // const float MAZE_WIDTH = 46.0f;
  // const float TIME_PER_CELL = 1.235f * 1000;
  const float MAZE_WIDTH = 60.0f;
  const float TIME_PER_CELL = 1.61f * 1000;
  const float TARGET_DISTANCE = MAZE_WIDTH / 2;
  Serial.begin(9600);
  Serial.println(" Starting...");
  motor.init(SPEED, TARGET_DISTANCE, TIME_PER_CELL, MAZE_WIDTH);

  bool wallLeft, wallStraight;
  float currentYaw = 0.0f;
  while (true) {
    motor.sense(&wallLeft, &wallStraight);
    act(wallLeft, wallStraight, &currentYaw);
  }
}

// Implements the logic of the left wall follower
void act(bool wallLeft, bool wallStraight, float *currentHeading) {
  // First check if we can turn left, as we always will when possible
  if (!wallLeft) {
    motor.turn(LEFT, 130, currentHeading);

    uint16_t previous = motor.ultrasonic.get_distance();

    // The path on the our left now is the one we came from. Hence, we do not have a wall to follow. Maybe we could use the right wall
    motor.straightLine(1, false);
    return;
  }
  // If it isn't possible, then check if we can continue straight
  if (!wallStraight) {
    // We have a wall on the left, so we can use it to ensure we are straight
    motor.straightLine(1, true);
    return;
  }
  // Lastly, if we cannot turn left or move forwards, then we will turn right
  motor.turn(RIGHT, 130, currentHeading);
}

void loop() {
}
