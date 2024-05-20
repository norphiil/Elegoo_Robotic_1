#include "device_ultrasonic.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////      ULTRASONIC     //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialises the ultrasonic sensor pins
 */
void Ultrasonic::init()
{
    pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_TRIG, OUTPUT);
}

float Ultrasonic::get_distance()
{
    digitalWrite(PIN_TRIG, LOW); // Preparing to send the ultrasonic pulse
    delayMicroseconds(2);        // Waiting for above pin to change

    digitalWrite(PIN_TRIG, HIGH); // Send the pulse
    delayMicroseconds(10);        // Wait for it to send

    digitalWrite(PIN_TRIG, LOW); // Go back to prepare mode

    long duration = pulseIn(PIN_ECHO, HIGH);

    if (duration > 0)
    {
        unsigned int distanceCm = (duration / 58);
        return distanceCm;
    }
    return 0;
}

/**
 * Sends a pulse to the ultrasonic sensor and measures the time it takes for the echo to return
 * Print the distance to the nearest object in centimeters
 */
void Ultrasonic::test()
{
    uint16_t min_distance = 300;
    uint16_t max_distance = 0;
    unsigned long start_time = millis();
    while (true)
    {
        digitalWrite(PIN_TRIG, LOW); // Preparing to send the ultrasonic pulse
        delayMicroseconds(2);        // Waiting for above pin to change

        digitalWrite(PIN_TRIG, HIGH); // Send the pulse
        delayMicroseconds(10);        // Wait for it to send

        digitalWrite(PIN_TRIG, LOW); // Go back to prepare mode

        long duration = pulseIn(PIN_ECHO, HIGH);

        /*
         * Speed of sound is 340m/s or 0.034cm/µs
         * The pulse travels to the object and back, so we divide by 2
         */
        if (duration > 0)
        {
            unsigned int distanceCm = (duration / 58);
            // if (distanceCm > 465)
            // {
            //     continue;
            // }
            // uint16_t distanceCm = ((duration * 0.034) / 2);
            if (distanceCm < min_distance)
            {
                min_distance = distanceCm;
            }
            if (distanceCm > max_distance)
            {
                max_distance = distanceCm;
            }
            if (millis() - start_time > 500)
            {
                start_time = millis();
                Serial.print("ultrasonic_sensor_test=");
                Serial.print(distanceCm);
                Serial.print("cm");
                Serial.print(" ");
                Serial.print(min_distance);
                Serial.print("cm");
                Serial.print(" ");
                Serial.print(max_distance);
                Serial.println("cm");
            }
        }
    }
}