#include "device_motor.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////         MOTOR          //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Initialises the motor pins
 * @param speed straight line speed
 * @param targetLeftDistance distance from left wall when assisted
 * @param timePerCell the time it takes the robot to cross cells at the given speed
 */
void Motor::init(int speed, float targetLeftDistance, float targetDistance, float timePerCell, uint8_t wallDistanceThreshold)
{
  this->speed = speed;
  this->targetLeftDistance = targetLeftDistance;
  this->timePerCell = timePerCell;
  this->wallDistanceThreshold = wallDistanceThreshold;

  pinMode(PIN_MOTOR_A_PWM, OUTPUT);
  pinMode(PIN_MOTOR_B_PWM, OUTPUT);
  pinMode(PIN_MOTOR_A_IN, OUTPUT);
  pinMode(PIN_MOTOR_B_IN, OUTPUT);
  pinMode(PIN_MOTOR_STBY, OUTPUT);

  // Enable the gyroaccel

  this->ultrasonic = Ultrasonic();
  this->ultrasonic.init();

  this->servo = Servo();
  this->servo.init();

  this->gyroaccel = GyroAccel();
  this->gyroaccel.init();

  // this->maze = Maze();
  // this->maze.init(6, 6);

  int cell_width = 30; // 30x30 cm

  // this->servo.setAngle(90);
  // int forwardDistance = this->ultrasonic.get_distance();

  // this->servo.setAngle(0);
  // int leftDistance = this->ultrasonic.get_distance();

  // this->servo.setAngle(180);
  // int rightDistance = this->ultrasonic.get_distance();

  // this->servo.setAngle(90);

  // Cell forwardCell = *maze.getCell(ceil(forwardDistance / cell_width), 0);
  // forwardCell.setTopWall(true);
  // maze.setCell(ceil(forwardDistance / cell_width), 0, &forwardCell);

  // Cell leftCell = *maze.getCell(0, ceil(leftDistance / cell_width));
  // leftCell.setLeftWall(true);
  // maze.setCell(0, ceil(leftDistance / cell_width), &leftCell);

  // Cell rightCell = *maze.getCell(0, ceil(rightDistance / cell_width));
  // rightCell.setRightWall(true);
  // maze.setCell(0, ceil(rightDistance / cell_width), &rightCell);

  // for (int i = 0; i < 6; i++)
  // {
  //     for (int j = 0; j < 6; j++)
  //     {
  //         Cell currentCell = *maze.getCell(i, j);
  //         currentCell.setVal(i * 6 + j);
  //         Serial.print("Cell (");
  //         Serial.print(i);
  //         Serial.print(" ");
  //         Serial.print(j);
  //         Serial.print(" ");
  //         Serial.print(currentCell.getVal());
  //         Serial.println(") ");
  //     }
  //     Serial.println();
  // }

  // Serial.println("---------------------------------------------------------");

  // for (int i = 0; i < 6; i++)
  // {
  //     for (int j = 0; j < 6 - 1; j++)
  //     {
  //         maze.setCell(i, j, maze.getCell(i, j + 1));
  //     }
  // }

  // for (int i = 0; i < 6; i++)
  // {
  //     for (int j = 0; j < 6; j++)
  //     {
  //         Cell currentCell = *maze.getCell(i, j);
  //         Serial.print("Cell (");
  //         Serial.print(i);
  //         Serial.print(" ");
  //         Serial.print(j);
  //         Serial.print(" ");
  //         Serial.print(currentCell.getVal());
  //         Serial.println(") ");
  //     }
  //     Serial.println();
  // }
}

/**
 * Returns the GyroAccel object
 */
GyroAccel *Motor::getGyroAccel()
{
  return &this->gyroaccel;
}

/**
 * Clamps the input speed between the MIN_SPEED and MAX_SPEED
 */
uint8_t Motor::normaliseSpeed(uint8_t speed)
{
  speed = speed > MIN_SPEED ? speed : MIN_SPEED;
  speed = speed < MAX_SPEED ? speed : MAX_SPEED;
  return speed;
}

/**
 * Changes the right motor's direction and speed
 */
void Motor::rightMotor(uint8_t direction, uint8_t speed)
{
  digitalWrite(PIN_MOTOR_A_IN, direction);
  analogWrite(PIN_MOTOR_A_PWM, speed);
}

/**
 * Changes the left motor's direction and speed
 */
void Motor::leftMotor(uint8_t direction, uint8_t speed)
{
  digitalWrite(PIN_MOTOR_B_IN, direction);
  analogWrite(PIN_MOTOR_B_PWM, speed);
}

/**
 * One function for all simple movements.
 */
void Motor::move(Direction direction, int speed_left, int speed_right)
{
  // Enable both motors
  digitalWrite(PIN_MOTOR_STBY, HIGH);

  // Clamp the speed between [MIN_SPEED, MAX_SPEED]
  // speed_left = this->normaliseSpeed(speed_left);
  // speed_right = this->normaliseSpeed(speed_right);

  // #ifdef DEBUG_MODE
  //     Serial.print("NORMALISED SPEED=");
  //     Serial.println(speed_left);
  //     Serial.println(speed_right);

  //     Serial.print("DIRECTION=");
  //     Serial.println(direction);
  // #endif

  switch (direction)
  {
  case FORWARDS:
    this->forwards(speed_left, speed_right);
    break;
  case BACKWARDS:
    this->backwards(speed_left, speed_right);
    break;
  case RIGHT:
    this->right(speed_left, speed_right);
    break;
  case LEFT:
    this->left(speed_left, speed_right);
    break;
  case FORWARDS_RIGHT:
    this->forwardsRight(speed_left, speed_right);
    break;
  case FORWARDS_LEFT:
    this->forwardsLeft(speed_left, speed_right);
    break;
  case BACKWARDS_RIGHT:
    this->backwardsRight(speed_left, speed_right);
    break;
  case BACKWARDS_LEFT:
    this->backwardsLeft(speed_left, speed_right);
    break;
  default:
    // In case of an unhandled direction, stop the motors, log the error
    this->stop();
    Serial.println("ERROR: INVALID DIRECTION");
  }
}

void Motor::forwards(int speed_left, int speed_right)
{
  uint8_t leftDir = speed_left > 0 ? MOTOR_FORWARDS : MOTOR_BACKWARDS;
  uint8_t rightDir = speed_right > 0 ? MOTOR_FORWARDS : MOTOR_BACKWARDS;

  // Serial.print("LEFT SPEED: ");
  // Serial.println(speed_left);
  // Serial.print("RIGHT SPEED: ");
  // Serial.println(speed_right);
  // Serial.println();
  // Serial.println();
  // Serial.println();
  // Serial.println();
  this->rightMotor(leftDir, abs(speed_right));
  this->leftMotor(rightDir, abs(speed_left));
}

void Motor::backwards(uint8_t speed_left, uint8_t speed_right)
{
  this->rightMotor(MOTOR_BACKWARDS, speed_right);
  this->leftMotor(MOTOR_BACKWARDS, speed_left);
}

void Motor::right(uint8_t speed_left, uint8_t speed_right)
{
  this->rightMotor(MOTOR_BACKWARDS, speed_right);
  this->leftMotor(MOTOR_FORWARDS, speed_left);
}

void Motor::left(uint8_t speed_left, uint8_t speed_right)
{
  this->rightMotor(MOTOR_FORWARDS, speed_right);
  this->leftMotor(MOTOR_BACKWARDS, speed_left);
}

void Motor::forwardsRight(uint8_t speed_left, uint8_t speed_right)
{
  this->rightMotor(MOTOR_FORWARDS, speed_right / 2);
  this->leftMotor(MOTOR_FORWARDS, speed_left);
}

void Motor::forwardsLeft(uint8_t speed_left, uint8_t speed_right)
{
  this->rightMotor(MOTOR_FORWARDS, speed_right);
  this->leftMotor(MOTOR_FORWARDS, speed_left / 2);
}

void Motor::backwardsRight(uint8_t speed_left, uint8_t speed_right)
{
  this->rightMotor(MOTOR_BACKWARDS, speed_right / 2);
  this->leftMotor(MOTOR_BACKWARDS, speed_left);
}

void Motor::backwardsLeft(uint8_t speed_left, uint8_t speed_right)
{
  this->rightMotor(MOTOR_BACKWARDS, speed_right);
  this->leftMotor(MOTOR_BACKWARDS, speed_left / 2);
}

void Motor::stop()
{
  analogWrite(PIN_MOTOR_A_PWM, 0);
  analogWrite(PIN_MOTOR_B_PWM, 0);
  digitalWrite(PIN_MOTOR_STBY, LOW);
}

void Motor::turn(Direction direction, uint8_t speed, float *currentHeading)
{
  // Calculate new heading based on the current heading and direction to turn
  float timeToTurn;
  float turnTime = 0.78;
  switch (direction)
  {
  case LEFT:
    timeToTurn = turnTime * 1000;
    break;
  case RIGHT:
    timeToTurn = turnTime * 1000;
    break;
  case BACKWARDS:
    timeToTurn = (turnTime * 2) * 1000;
    break;
  }
  unsigned long currentTime = millis();
  while (millis() - currentTime < timeToTurn)
  {
    this->move(direction, speed, speed);
  }
  this->stop();
}

void Motor::updateSensor(void)
{
  float roll, pitch, yaw;
  this->gyroaccel.getRotation(&roll, &pitch, &yaw);
}

void Motor::sense(bool *wallLeft, bool *wallStraight)
{
  this->servo.setAngle(200);
  delay(50);
  *wallLeft = this->ultrasonic.get_distance() <= this->wallDistanceThreshold;

  this->servo.setAngle(90);
  delay(50);
  *wallStraight = this->ultrasonic.get_distance() <= this->wallDistanceThreshold;
  this->servo.setAngle(200);
  delay(15);
}

/**
 * Moves the robot in a straight line at a given speed
 * @param speed The speed at which to move (0-255 range)
 * @param targetYaw The target yaw angle of the robot
 */
void Motor::straightLine(float totalDist, bool assisted)
{
  unsigned long lastTime = millis();
  // Adjusted PID constants
  const float Kp = 6.2f; // Proportional gain
  const float Ki = 0.5f; // Integral gain
  const float Kd = 1.0f; // Derivative gain

  const int8_t maxSpeed = 50; // Maximum speed

  float errorDist = 0.0f;
  float integralDist = 0.0f;
  float derivativeDist = 0.0f;
  float previousDist = 0.0f;

  const float integralMax = 1.0f;
  const float integralMin = -1.0f;

  this->servo.setAngle(90);
  uint16_t forwardDistance = this->ultrasonic.get_distance();

  while (forwardDistance > 2)
  {
    this->servo.setAngle(180);
    uint16_t leftDistance = this->ultrasonic.get_distance();

    unsigned long currentTime = millis();
    const int delta = currentTime - lastTime;
    lastTime = currentTime;

    float roll, pitch, currentYaw;
    this->gyroaccel.getRotation(&roll, &pitch, &currentYaw);

    errorDist = targetDist - leftDistance;
    Serial.println(errorDist);
    integralDist += errorDist * delta;
    if (integralDist > integralMax)
    {
      integralDist = integralMax;
    }
    else if (integralDist < integralMin)
    {
      integralDist = integralMin;
    }

    derivativeDist = (targetDist - previousDist) / delta;

    previousDist = leftDistance;

    float pidOutput = -Kp * errorDist - Ki * integralDist - Kd * derivativeDist;

    int8_t leftSpeed = speed + pidOutput;
    if (leftSpeed > maxSpeed)
    {
      leftSpeed = maxSpeed;
    }
    else if (leftSpeed < -maxSpeed)
    {
      leftSpeed = -maxSpeed;
    }
    int8_t rightSpeed = speed - pidOutput;
    if (rightSpeed > maxSpeed)
    {
      rightSpeed = maxSpeed;
    }
    else if (rightSpeed < -maxSpeed)
    {
      rightSpeed = -maxSpeed;
    }

    this->move(FORWARDS, leftSpeed, rightSpeed);

    this->servo.setAngle(90);
    uint16_t forwardDistance = this->ultrasonic.get_distance();
  }
  this->stop();
}

void Motor::straightUnassisted(Direction direction = FORWARDS)
{
  // 5 units of velocity added to the left to reduce the natural straying from the centre
  this->move(direction, this->speed + 5, this->speed);
}

// returns true while there is still a wall to track
bool Motor::straightAssisted(unsigned long *lastTime, uint16_t distance)
{
  // 1.8
  const float Kp = 8.0f; // Proportional gain
  const float Kd = 2.0f; // Derivative gain

  float CTE = 0.0f;
  float dCTE = 0.0f;
  float previousCTE = 0.0f;
  float CTEdifference = 0.0f;

  // Get current time and delta
  unsigned long currentTime = millis();
  const int delta = currentTime - *lastTime;
  *lastTime = currentTime;

  CTE = distance - this->targetLeftDistance;
  // Derivate of error
  CTEdifference = CTE - previousCTE;
  dCTE = CTEdifference / delta;

  // Calculate PD output
  int pdOutput = CTE * Kp + dCTE * Kd;
  // Adjust speeds and move accordingly
  this->move(FORWARDS, this->speed - pdOutput, this->speed + pdOutput);
  this->updateSensor();
  return true;
}
