#include "devices.h"

// Change to 0 or comment this line out to switch off debug mode and hide Serial prints
// #define DEBUG_MODE 1

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////         MOTOR          //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Motor::init()
{
  pinMode(PIN_MOTOR_A_PWM, OUTPUT);
  pinMode(PIN_MOTOR_B_PWM, OUTPUT);
  pinMode(PIN_MOTOR_A_IN, OUTPUT);
  pinMode(PIN_MOTOR_B_IN, OUTPUT);
  pinMode(PIN_MOTOR_STBY, OUTPUT);
}

uint8_t Motor::normaliseSpeed(uint8_t speed)
{
  speed = speed > MIN_SPEED ? speed : MIN_SPEED;
  speed = speed < MAX_SPEED ? speed : MAX_SPEED;
  return speed;
}

void Motor::rightMotor(uint8_t direction, uint8_t speed)
{
  Serial.print("RIGHT MOTOR MOVING: ");
  Serial.println(direction);
  digitalWrite(PIN_MOTOR_A_IN, direction);
  analogWrite(PIN_MOTOR_A_PWM, speed);
}

void Motor::leftMotor(uint8_t direction, uint8_t speed)
{
  digitalWrite(PIN_MOTOR_B_IN, direction);
  analogWrite(PIN_MOTOR_B_PWM, speed);
}

void Motor::move(Direction direction, uint8_t speed)
{
  // Enable both motors
  digitalWrite(PIN_MOTOR_STBY, HIGH);

  // Clamp the speed between [MIN_SPEED, MAX_SPEED]
  speed = this->normaliseSpeed(speed);

#ifdef DEBUG_MODE
  Serial.print("NORMALISED SPEED=");
  Serial.println(speed);

  Serial.print("DIRECTION=");
  Serial.println(direction);
#endif

  switch (direction)
  {
  case FORWARDS:
    this->forwards(speed);
    break;
  case BACKWARDS:
    this->backwards(speed);
    break;
  case RIGHT:
    this->right(speed);
    break;
  case LEFT:
    this->left(speed);
    break;
  case FORWARDS_RIGHT:
    this->forwardsRight(speed);
    break;
  case FORWARDS_LEFT:
    this->forwardsLeft(speed);
    break;
  case BACKWARDS_RIGHT:
    this->backwardsRight(speed);
    break;
  case BACKWARDS_LEFT:
    this->backwardsLeft(speed);
    break;
  default:
    // In case of an unhandled direction, stop the motors, log the error
    this->stop();
    Serial.println("ERROR: INVALID DIRECTION");
  }
}

void Motor::forwards(uint8_t speed)
{
  this->rightMotor(MOTOR_FORWARDS, speed);
  this->leftMotor(MOTOR_FORWARDS, speed);
}

void Motor::backwards(uint8_t speed)
{
  this->rightMotor(MOTOR_BACKWARDS, speed);
  this->leftMotor(MOTOR_BACKWARDS, speed);
}

void Motor::right(uint8_t speed)
{
  this->rightMotor(MOTOR_BACKWARDS, speed);
  this->leftMotor(MOTOR_FORWARDS, speed);
}

void Motor::left(uint8_t speed)
{
  this->rightMotor(MOTOR_FORWARDS, speed);
  this->leftMotor(MOTOR_BACKWARDS, speed);
}

void Motor::forwardsRight(uint8_t speed)
{
  this->rightMotor(MOTOR_FORWARDS, speed / 2);
  this->leftMotor(MOTOR_FORWARDS, speed);
}

void Motor::forwardsLeft(uint8_t speed)
{
  this->rightMotor(MOTOR_FORWARDS, speed);
  this->leftMotor(MOTOR_FORWARDS, speed / 2);
}

void Motor::backwardsRight(uint8_t speed)
{
  this->rightMotor(MOTOR_BACKWARDS, speed / 2);
  this->leftMotor(MOTOR_BACKWARDS, speed);
}

void Motor::backwardsLeft(uint8_t speed)
{
  this->rightMotor(MOTOR_BACKWARDS, speed);
  this->leftMotor(MOTOR_BACKWARDS, speed / 2);
}

void Motor::stop()
{
  analogWrite(PIN_MOTOR_A_PWM, 0);
  analogWrite(PIN_MOTOR_B_PWM, 0);
  digitalWrite(PIN_MOTOR_STBY, LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////      ULTRASONIC     //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Ultrasonic::init()
{
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
}

void Ultrasonic::test()
{
  digitalWrite(PIN_TRIG, LOW); // Preparing to send the ultrasonic pulse
  delayMicroseconds(2);        // Waiting for above pin to change

  digitalWrite(PIN_TRIG, HIGH); // Send the pulse
  delayMicroseconds(10);        // Wait for it to send

  digitalWrite(PIN_TRIG, LOW); // Go back to prepare mode

  /* Takes the time in microseconds for the sound to travel TO AND FROM the object from the echo pin.
   * Divided by 58 to convert to cm. 29 is the approximate time taken for sound to travel 1cm. The sound
   * travels both TO and FROM the object, so twice 29 is 58.
   */
  unsigned int distanceCm = ((unsigned int)pulseIn(PIN_ECHO, HIGH) / 58);
  Serial.print("ultrasonic_sensor_test=");
  Serial.print(distanceCm);
  Serial.println("cm");
}

uint16_t Ultrasonic::get()
{
  digitalWrite(PIN_TRIG, LOW); // Preparing to send the ultrasonic pulse
  delayMicroseconds(2);        // Waiting for above pin to change

  digitalWrite(PIN_TRIG, HIGH); // Send the pulse
  delayMicroseconds(10);        // Wait for it to send

  digitalWrite(PIN_TRIG, LOW); // Go back to prepare mode

  /* Takes the time in microseconds for the sound to travel TO AND FROM the object from the echo pin.
   * Divided by 58 to convert to cm. 29 is the approximate time taken for sound to travel 1cm. The sound
   * travels both TO and FROM the object, so twice 29 is 58.
   */
  unsigned int distanceCm = ((unsigned int)pulseIn(PIN_ECHO, HIGH) / 58);
  return distanceCm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////      IR      //////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void IR::init()
{
  pinMode(PIN_IR_L, INPUT);
  pinMode(PIN_IR_M, INPUT);
  pinMode(PIN_IR_R, INPUT);

  // Set default threshold
  this->setThreshold(BLACK_ON_WHITE_THRESHOLDS);
}

void IR::setThreshold(uint16_t threshold[3])
{
  this->thresholdL = threshold[0];
  this->thresholdM = threshold[1];
  this->thresholdR = threshold[2];
}

void IR::setThreshold(uint16_t thresholdL, uint16_t thresholdM, uint16_t thresholdR)
{
  this->thresholdL = thresholdL;
  this->thresholdM = thresholdM;
  this->thresholdR = thresholdR;
}

void IR::test()
{
  int ir_value = analogRead(PIN_IR_L);
  Serial.print("IR_L=");
  Serial.println(ir_value);
  ir_value = analogRead(PIN_IR_M);
  Serial.print("IR_M=");
  Serial.println(ir_value);
  ir_value = analogRead(PIN_IR_R);
  Serial.print("IR_R=");
  Serial.println(ir_value);
}

bool IR::get(IRSensor sensor)
{
  // Takes the reading of the given sensor and checks if it exceeds the threshold, returning true in that case
  // to signify that the sensor detects a line
  switch (sensor)
  {
  case IR_LEFT:
    return analogRead(PIN_IR_L) >= this->thresholdL;
  case IR_MIDDLE:
    return analogRead(PIN_IR_M) >= this->thresholdM;
  case IR_RIGHT:
    return analogRead(PIN_IR_R) >= this->thresholdR;
  default:
    return -1;
  }
}

uint16_t IR::value(IRSensor sensor)
{
  switch (sensor)
  {
  case IR_LEFT:
    return analogRead(PIN_IR_L);
  case IR_MIDDLE:
    return analogRead(PIN_IR_M);
  case IR_RIGHT:
    return analogRead(PIN_IR_R);
  default:
    return -1;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////      SERVO     ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Servo::init()
{
  Serial.println("Servo init");
  // Servo motor
  pinMode(PIN_SERVO, OUTPUT); // Set servo pin as output
  delay(1000);
  this->angle = 90;
  this->setAngleBrute(90);
}

/**
 * Turns the servo motor to a given angle (0-180)
 */
void Servo::setAngle(uint8_t new_angle)
{
  if (this->angle - new_angle > 0)
  {
    for (int i = this->angle; i > new_angle; i--)
    {
      this->setAngleBrute(i);
      delay(1);
    }
  }
  else
  {
    for (int i = this->angle; i < new_angle; i++)
    {
      this->setAngleBrute(i);
      delay(1);
    }
  }
}

/**
 * Turns the servo motor to a given angle (0-180)
 */
void Servo::setAngleBrute(uint8_t new_angle)
{
  uint8_t tmp_angle = new_angle - 2;
  tmp_angle = max(0, new_angle);
  tmp_angle = min(180, new_angle);
  this->angle = new_angle;
  // Convert degrees to pulse width
  int pulseWidth = map(tmp_angle, 0, 180, 500, 2400);
  // Set servo position
  digitalWrite(PIN_SERVO, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(PIN_SERVO, LOW);
}