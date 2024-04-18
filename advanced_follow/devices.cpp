#include "devices.h"

// Change to 0 or comment this line out to switch off debug mode and hide Serial prints
// #define DEBUG_MODE 1

#define LINE_FOLLOW_SPEED 50
#define TURN_SPEED 50
#define TURN_DURATION 200

#define LED_PIN 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS]; // Define the LED array

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
  this->gyroAccel.init();
  this->ir.init();
  this->ultrasonic.init();
  this->servo.init();

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);
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

  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit, LowerLimit;
  Kp = 10;
  UpperLimit = 255;
  LowerLimit = 10;

  switch (direction)
  {
  case FORWARDS:
    this->linearMotionControl(FORWARDS, directionRecord, speed, Kp, UpperLimit, LowerLimit);
    directionRecord = 1;
    break;
  case BACKWARDS:
    this->linearMotionControl(BACKWARDS, directionRecord, speed, Kp, UpperLimit, LowerLimit);
    directionRecord = 2;
    break;
  case LEFT:
    this->left(speed);
    directionRecord = 3;
    break;
  case RIGHT:
    this->right(speed);
    directionRecord = 4;
    break;
  case FORWARDS_LEFT:
    this->forwardsLeft(speed);
    break;
    directionRecord = 5;
  case BACKWARDS_LEFT:
    this->backwardsLeft(speed);
    directionRecord = 6;
    break;
  case FORWARDS_RIGHT:
    this->forwardsRight(speed);
    directionRecord = 7;
    break;
  case BACKWARDS_RIGHT:
    this->backwardsRight(speed);
    directionRecord = 8;
    break;
  case STOP:
    this->stop();
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

void Motor::motorControl(uint8_t direction_A, uint8_t speed_A, uint8_t direction_B, uint8_t speed_B, uint8_t controlED)
{
  if (controlED == true)
  {
    this->rightMotor(direction_A, speed_A);
    this->leftMotor(direction_B, speed_B);
  }
  else
  {
    this->stop();
  }
}

void Motor::stop()
{
  analogWrite(PIN_MOTOR_A_PWM, 0);
  analogWrite(PIN_MOTOR_B_PWM, 0);
  digitalWrite(PIN_MOTOR_STBY, LOW);
}

void Motor::linearMotionControl(uint8_t direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit, uint8_t LowerLimit)
{

  static float Roll, Pitch, Yaw;
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    this->motorControl(/*direction_A*/ 3, /*speed_A*/ 0,
                       /*direction_B*/ 3, /*speed_B*/ 0, /*controlED*/ true); // Motor control
    this->gyroAccel.getRotation(&Roll, &Pitch, &Yaw);
    is_time = millis();
  }
  // if (en != directionRecord)
  if (en != directionRecord)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  // 加入比例常数Kp
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < LowerLimit)
  {
    R = LowerLimit;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < LowerLimit)
  {
    L = LowerLimit;
  }
  if (direction == FORWARDS) // 前进
  {
    this->motorControl(/*direction_A*/ true, /*speed_A*/ R,
                       /*direction_B*/ true, /*speed_B*/ L, /*controlED*/ true);
  }
  else if (direction == BACKWARDS) // 后退
  {
    this->motorControl(/*direction_A*/ false, /*speed_A*/ L,
                       /*direction_B*/ false, /*speed_B*/ R, /*controlED*/ true);
  }
}

void Motor::flashLed(void)
{
  leds[0] = CRGB::HotPink;
  FastLED.show(); // Update the LED strip with the new color
  delay(500);     // Wait for 500 milliseconds

  // Turn off the LED
  leds[0] = CRGB::Black;
  FastLED.show(); // Update the LED strip
}

void Motor::startTracking(void)
{
  static bool timestamp = true;
  static bool BlindDetection = true;
  static unsigned long MotorRL_time = 0;

  float Roll, Pitch, Yaw;
  this->gyroAccel.getRotation(&Roll, &Pitch, &Yaw);

  bool leftSensor = ir.get(IR_LEFT);
  bool middleSensor = ir.get(IR_MIDDLE);
  bool rightSensor = ir.get(IR_RIGHT);

  if (leftSensor && middleSensor && rightSensor)
  {
    this->isIntersectionCounter++;
    this->move(FORWARDS, LINE_FOLLOW_SPEED);
    if (this->isIntersectionCounter >= 2)
    {
      this->isIntersectionCounter = 0;
      this->stop();
      this->servo.setAngle(90);
      unsigned int distance_F = this->ultrasonic.get();
      flashLed();
      this->servo.setAngle(180);
      unsigned int distance_L = this->ultrasonic.get();
      flashLed();
      this->servo.setAngle(0);
      unsigned int distance_R = this->ultrasonic.get();
      flashLed();
      if (distance_L > distance_F && distance_L > distance_R)
      {
        lastDirection = LEFT;
        this->move(LEFT, LINE_FOLLOW_SPEED);
        delay(TURN_DURATION);
        this->move(FORWARDS, LINE_FOLLOW_SPEED);
        delay(TURN_DURATION);
        while (!ir.get(IR_MIDDLE))
        {
          this->move(LEFT, LINE_FOLLOW_SPEED);
        }
      }
      else if (distance_R > distance_F && distance_R > distance_L)
      {
        lastDirection = RIGHT;
        this->move(RIGHT, LINE_FOLLOW_SPEED);
        delay(TURN_DURATION);
        this->move(FORWARDS, LINE_FOLLOW_SPEED);
        delay(TURN_DURATION);
        while (!ir.get(IR_MIDDLE))
        {
          this->move(RIGHT, LINE_FOLLOW_SPEED);
        }
      }
      else
      {
        lastDirection = FORWARDS;
        this->move(FORWARDS, LINE_FOLLOW_SPEED);
      }
      this->servo.setAngle(90);
      this->move(FORWARDS, LINE_FOLLOW_SPEED);
      delay(10);
    }
  }
  else if (middleSensor)
  {
    this->move(FORWARDS, 100);
    timestamp = true;
    BlindDetection = true;
    lastDirection = FORWARDS;
  }
  else if (rightSensor)
  {
    this->move(RIGHT, 100);
    timestamp = true;
    BlindDetection = true;
    lastDirection = RIGHT;
  }
  else if (leftSensor)
  {
    this->move(LEFT, 100);
    timestamp = true;
    BlindDetection = true;
    lastDirection = LEFT;
  }
  else
  {
    if (timestamp == true)
    {
      timestamp = false;
      MotorRL_time = millis();
      this->move(STOP, 0);
    }
    unsigned long m = millis();
    if (lastDirection == FORWARDS)
    {
      this->stop();
      BlindDetection = true;
    }
    else if ((this->ir.inInterval((m - MotorRL_time), 0, 200) || this->ir.inInterval((m - MotorRL_time), 1600, 2000)) && BlindDetection == true)
    {
      this->move(RIGHT, 100);
    }
    else if (((this->ir.inInterval((m - MotorRL_time), 200, 1600))) && BlindDetection == true)
    {
      this->move(LEFT, 100);
    }
    else if ((this->ir.inInterval((m - MotorRL_time), 3000, 3500)))
    {
      BlindDetection = false;
      this->move(STOP, 0);
    }
  }
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

static bool IR::inInterval(long x, long s, long e) // f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
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
  for (int i = 0; i < 180; i++)
  {
    this->setAngleBrute(i);
    delay(1);
  }
  this->setAngle(90);
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