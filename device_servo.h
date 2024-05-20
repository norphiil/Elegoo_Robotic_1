#include <Arduino.h>

class Servo
{
public:
    void init(void);
    void test(void);
    void setAngle(uint8_t new_angle);
    // uint8_t find_way(Ultrasonic ultrasonic);

private:
    void setAngleBrute(uint8_t new_angle);
    uint8_t angle;

#define PIN_SERVO 10 // Pin responsible for the servo motor
};