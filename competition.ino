#include <Arduino.h>
#include "device_motor.h"
#include <avr/wdt.h>
#include "device_maze.h"
#include <Vector.h>

Maze maze;
Motor motor;
float initialYaw = 0;

unsigned long lastTime = 0;
const float MAZE_WIDTH = 30.0f;

void setup()
{
  const int SPEED = 100;
  // const float MAZE_WIDTH = 46.0f;
  // const float TIME_PER_CELL = 1.235f * 1000;
  const float TIME_PER_CELL = 0.805f * 1000;
  const float TARGET_LEFT_DISTANCE = 13.5;
  const float TARGET_FORWARD_DISTANCE = 13.5;
  Serial.begin(9600);
  Serial.println(" Starting...");
  motor.init(SPEED, TARGET_LEFT_DISTANCE, TARGET_FORWARD_DISTANCE, TIME_PER_CELL, MAZE_WIDTH / 2);
  motor.updateSensor();

  bool wallLeft, wallStraight;
  float currentYaw = 0.0f;
  while (true)
  {
    motor.sense(&wallLeft, &wallStraight);
    act(wallLeft, wallStraight, &currentYaw);
  }
  // maze.update(NORTH, false, false);
  // maze.move(WEST);
  // maze.update(WEST, true, true);
  // maze.update(NORTH, true, true);
  // maze.update(EAST, true, false);
  // maze.move(EAST);
  // maze.update(EAST, false, true);
  // maze.move(NORTH);
  // maze.update(NORTH, true, true);
  // maze.update(EAST, true, true);
  // maze.update(SOUTH, true, false);
  // Vector<Heading> neighbours = maze.start->neighbours;
  // Serial.print("Size: ");
  // Serial.println(neighbours.size());
}

// Implements the logic of the left wall follower
void act(bool wallLeft, bool wallStraight, float *currentHeading)
{
  // First check if we can turn left, as we always will when possible
  if (!wallLeft)
  {
    motor.turn(LEFT, 70, currentHeading);

    // The path on the our left now is the one we came from. Hence, we do not have a wall to follow. Maybe we could use the right wall
    motor.servo.setAngle(90);
    delay(10);
    float forwardDistance = motor.ultrasonic.get_distance();
    motor.straightLine(forwardDistance, false);
    return;
  }
  // If it isn't possible, then check if we can continue straight
  if (!wallStraight)
  {
    // We have a wall on the left, so we can use it to ensure we are straight
    motor.servo.setAngle(90);
    delay(10);
    float forwardDistance = motor.ultrasonic.get_distance();
    motor.straightLine(forwardDistance, true);
    return;
  }
  // Lastly, if we cannot turn left or move forwards, then we will turn right
  motor.turn(RIGHT, 70, currentHeading);
}

void loop()
{
}
