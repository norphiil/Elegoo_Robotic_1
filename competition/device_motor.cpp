#include "device_motor.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////         MOTOR          //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Initialises the motor pins
 * @param speed straight line speed
 * @param targetDistance distance from left wall when assisted
 * @param timePerCell the time it takes the robot to cross cells at the given speed
 */
void Motor::init(int speed, float targetDistance, float timePerCell, uint8_t wallDistanceThreshold) {
  this->speed = speed;
  this->targetDistance = targetDistance;
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

  int cell_width = 30;  // 30x30 cm

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
GyroAccel *Motor::getGyroAccel() {
  return &this->gyroaccel;
}

/**
 * Clamps the input speed between the MIN_SPEED and MAX_SPEED
 */
uint8_t Motor::normaliseSpeed(uint8_t speed) {
  speed = speed > MIN_SPEED ? speed : MIN_SPEED;
  speed = speed < MAX_SPEED ? speed : MAX_SPEED;
  return speed;
}

/**
 * Changes the right motor's direction and speed
 */
void Motor::rightMotor(uint8_t direction, uint8_t speed) {
  digitalWrite(PIN_MOTOR_A_IN, direction);
  analogWrite(PIN_MOTOR_A_PWM, speed);
}

/**
 * Changes the left motor's direction and speed
 */
void Motor::leftMotor(uint8_t direction, uint8_t speed) {
  digitalWrite(PIN_MOTOR_B_IN, direction);
  analogWrite(PIN_MOTOR_B_PWM, speed);
}

/**
 * One function for all simple movements.
 */
void Motor::move(Direction direction, int speed_left, int speed_right) {
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

  switch (direction) {
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

void Motor::forwards(int speed_left, int speed_right) {
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

void Motor::backwards(uint8_t speed_left, uint8_t speed_right) {
  this->rightMotor(MOTOR_BACKWARDS, speed_right);
  this->leftMotor(MOTOR_BACKWARDS, speed_left);
}

void Motor::right(uint8_t speed_left, uint8_t speed_right) {
  this->rightMotor(MOTOR_BACKWARDS, speed_right);
  this->leftMotor(MOTOR_FORWARDS, speed_left);
}

void Motor::left(uint8_t speed_left, uint8_t speed_right) {
  this->rightMotor(MOTOR_FORWARDS, speed_right);
  this->leftMotor(MOTOR_BACKWARDS, speed_left);
}

void Motor::forwardsRight(uint8_t speed_left, uint8_t speed_right) {
  this->rightMotor(MOTOR_FORWARDS, speed_right / 2);
  this->leftMotor(MOTOR_FORWARDS, speed_left);
}

void Motor::forwardsLeft(uint8_t speed_left, uint8_t speed_right) {
  this->rightMotor(MOTOR_FORWARDS, speed_right);
  this->leftMotor(MOTOR_FORWARDS, speed_left / 2);
}

void Motor::backwardsRight(uint8_t speed_left, uint8_t speed_right) {
  this->rightMotor(MOTOR_BACKWARDS, speed_right / 2);
  this->leftMotor(MOTOR_BACKWARDS, speed_left);
}

void Motor::backwardsLeft(uint8_t speed_left, uint8_t speed_right) {
  this->rightMotor(MOTOR_BACKWARDS, speed_right);
  this->leftMotor(MOTOR_BACKWARDS, speed_left / 2);
}

void Motor::stop() {
  analogWrite(PIN_MOTOR_A_PWM, 0);
  analogWrite(PIN_MOTOR_B_PWM, 0);
  digitalWrite(PIN_MOTOR_STBY, LOW);
}

void Motor::turn(Direction direction, uint8_t speed, float *currentHeading) {
  // Calculate new heading based on the current heading and direction to turn
  float newHeading = *currentHeading;
  switch (direction) {
    case LEFT:
      newHeading -= 90.0f;
      break;
    case RIGHT:
      newHeading += 90.0f;
      break;
    case BACKWARDS:
      newHeading += 180.0f;
      break;
  }

  newHeading = fmodf(newHeading, 360.0f);
  if (newHeading < 0) newHeading += 360.0f;

  // Calculate the shortest angle to turn
  float angleDifference = fmodf(newHeading - *currentHeading + 360.0f, 360.0f);
  float reverseAngleDifference = 360.0f - angleDifference;
  Direction rotationDirection = (angleDifference <= reverseAngleDifference) ? RIGHT : LEFT;
  angleDifference = angleDifference <= reverseAngleDifference ? angleDifference : reverseAngleDifference;

  double tolerance = rotationDirection == RIGHT ? 6.75f : 1.75f;

  // Perform the turn
  while (!gyroaccel.areAnglesEqual(newHeading, *currentHeading, tolerance)) {
    // Adjust speed based on angle
    uint8_t adjustedSpeed = speed;
    if (angleDifference < 180) {
      adjustedSpeed = static_cast<uint8_t>((angleDifference / 180.0f) * speed);
    }

    move(rotationDirection, adjustedSpeed, adjustedSpeed);
    this->gyroaccel.getRotation(nullptr, nullptr, currentHeading);
  }
  stop();
  *currentHeading = newHeading;
}

void Motor::sense(bool *wallLeft, bool *wallStraight) {
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
void Motor::straightLine(uint8_t cells, bool assisted) {
  // Look left
  this->servo.setAngle(200);

  // Get start time
  unsigned long ogTime = millis();
  unsigned long lastTime = ogTime;

  bool leftWall = true;

  while (millis() - ogTime < this->timePerCell * cells) {
    // If and when the left wall ends (e.g. a left turn is coming up in the cell we just moved into), switch to unassisted mode
    if (assisted && leftWall) {
      leftWall = this->straightAssisted(cells, &lastTime);
    } else {
      this->straightUnassisted(cells);
    }
  }

  // Braking phase, makes stopping smoother and more consistent
  this->backwards(50, 50);
  delay(50);
  this->stop();
}

void Motor::straightUnassisted(uint8_t cells) {
  // 5 units of velocity added to the left to reduce the natural straying from the centre
  this->move(FORWARDS, this->speed + 5, this->speed);
}

// returns true while there is still a wall to track
bool Motor::straightAssisted(uint8_t cells, unsigned long *lastTime) {
  // 1.8
  const float Kp = 2.0f;  // Proportional gain
  const float Kd = 1.0f;  // Derivative gain

  float CTE = 0.0f;
  float dCTE = 0.0f;
  float previousCTE = 0.0f;
  float CTEdifference = 0.0f;



  // Get current time and delta
  unsigned long currentTime = millis();
  const int delta = currentTime - *lastTime;
  *lastTime = currentTime;

  // Get current error
  uint16_t distance = this->ultrasonic.get_distance();

  if (distance > this->wallDistanceThreshold) {
    return false;
  }

  CTE = distance - this->targetDistance;
  // Derivate of error
  CTEdifference = CTE - previousCTE;
  dCTE = CTEdifference / delta;

  // Calculate PD output
  int pdOutput = CTE * Kp + dCTE * Kd;
  // Adjust speeds and move accordingly
  this->move(FORWARDS, this->speed - pdOutput, this->speed + pdOutput);
  return true;
}
