#include "devices.h"

#define DEBUG_MODE 1

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
    Serial.print("moving forwards");
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