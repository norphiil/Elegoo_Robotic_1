#include "device_servo.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////      SERVO     ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Servo::init()
{
    Serial.println("Servo init");
    // Servo motor
    pinMode(PIN_SERVO, OUTPUT); // Set servo pin as output
    this->angle = 90;
    this->setAngle(90);
}

/**
 * Turns the servo motor to a given angle (0-180)
 */
void Servo::setAngle(uint8_t new_angle, uint8_t step = 2)
{
    if (this->angle > new_angle)
    {
        for (int i = this->angle; i > new_angle; i -= step)
        {
            if (i - step < new_angle)
            {
                i = new_angle;
            }
            this->setAngleBrute(i);
            delay(1); // Assuming you want the delay back
        }
    }
    else
    {
        for (int i = this->angle; i < new_angle; i += step)
        {
            if (i + step > new_angle)
            {
                i = new_angle;
            }
            this->setAngleBrute(i);
            delay(1); // Assuming you want the delay back
        }
    }
    // Set the final angle to ensure it's exactly the new_angle
    this->setAngleBrute(new_angle);
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