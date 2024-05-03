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

uint16_t Ultrasonic::get_distance()
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

// void Ultrasonic::calculate_field_of_view(Servo servo)
// {

//     auto calculate_distance_BC = [](double AB, double angle_BAC, double AC)
//     {
//         double angle_rad = angle_BAC * M_PI / 180.0; // Convert angle to radians
//         double BC_squared = pow(AB, 2) + pow(AC, 2) - 2 * AB * AC * cos(angle_rad);
//         return sqrt(BC_squared);
//     };
//     auto get_distant_and_angle = [](uint16_t *object_distance, uint16_t *last_object_distance, uint8_t *angle, int i)
//     {
//         digitalWrite(PIN_TRIG, LOW); // Preparing to send the ultrasonic pulse
//         delayMicroseconds(2);        // Waiting for above pin to change

//         digitalWrite(PIN_TRIG, HIGH); // Send the pulse
//         delayMicroseconds(10);        // Wait for it to send

//         digitalWrite(PIN_TRIG, LOW); // Go back to prepare mode

//         long duration = pulseIn(PIN_ECHO, HIGH);

//         /*
//          * Speed of sound is 340m/s or 0.034cm/µs
//          * The pulse travels to the object and back, so we divide by 2
//          */
//         if (duration > 0)
//         {
//             unsigned int distanceCm = (duration / 58);
//             if (distanceCm > 465)
//             {
//             }
//             else
//             {

//                 if (*object_distance == 0)
//                 {
//                     *object_distance = distanceCm;
//                 }
//                 else
//                 {
//                     int tmp_distance = *object_distance;
//                     Serial.print("distanceCm: ");
//                     Serial.println(distanceCm);
//                     Serial.print("object_distance: ");
//                     Serial.println(tmp_distance);
//                     if (abs(distanceCm - tmp_distance) > 20)
//                     {
//                         *angle = i;
//                     }
//                     else
//                     {
//                         *last_object_distance = distanceCm;
//                     }
//                 }
//             }
//         }
//     };
//     uint16_t left_object_distance = 0;
//     uint16_t last_left_object_distance = 0;
//     uint16_t right_object_distance = 0;
//     uint16_t last_right_object_distance = 0;
//     uint8_t left_angle = 0;
//     uint8_t right_angle = 0;
//     unsigned long start_time = millis();
//     for (int i = 90;
//          i < 180; i++)
//     {
//         servo.setAngle(i);
//         delay(15);
//         get_distant_and_angle(&right_object_distance, &last_right_object_distance, &right_angle, i);
//         delay(15);
//         if (right_angle != 0)
//         {
//             break;
//         }
//     }
//     servo.setAngle(90);
//     delay(1000);
//     for (int i = 90;
//          i > 0; i--)
//     {
//         servo.setAngle(i);
//         delay(15);
//         get_distant_and_angle(&left_object_distance, &last_left_object_distance, &left_angle, i);
//         delay(15);
//         if (left_angle != 0)
//         {
//             break;
//         }
//     }
//     servo.setAngle(90);

//     Serial.print("left_test=");
//     Serial.print(left_object_distance);
//     Serial.print("cm");
//     Serial.print(" ");
//     Serial.print(left_angle);
//     Serial.print("°");
//     Serial.print(" ");
//     Serial.print(calculate_distance_BC(left_object_distance, abs(left_angle - 90), last_left_object_distance));
//     Serial.println("cm");
//     Serial.print("right_test=");
//     Serial.print(right_object_distance);
//     Serial.print("cm");
//     Serial.print(" ");
//     Serial.print(right_angle);
//     Serial.print("°");
//     Serial.print(" ");
//     Serial.print(calculate_distance_BC(right_object_distance, abs(right_angle - 90), last_right_object_distance));
//     Serial.println("cm");
// }