#include "device_servo.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////      SERVO     ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Servo::init()
{
    Serial.println("Servo init");
    // Servo motor
    pinMode(PIN_SERVO, OUTPUT); // Set servo pin as output
    delay(1000);
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
            // delay(15);
        }
    }
    else
    {
        for (int i = this->angle; i < new_angle; i++)
        {
            this->setAngleBrute(i);
            // delay(15);
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

void Servo::test()
{
    this->setAngle(0);
    delay(1000);
    this->setAngle(180);
    delay(1000);
    this->setAngle(90);
}