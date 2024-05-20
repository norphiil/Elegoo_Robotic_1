#include <Arduino.h>

class Ultrasonic
{
public:
    void init(void);
    // test to get the distance every second
    void test(void);
    // void get(uint16_t *get);
    // void calculate_field_of_view(Servo servo);
    float get_distance(void);

private:
    // unsigned int microseconds_to_cm(unsigned int microseconds);
#define PIN_TRIG 13      // Pin responsible for the trigger pulse. LOW = prepare for pulse, HIGH = send pulse
#define PIN_ECHO 12      // Pin which gives us the time until the echo pulse was received, HIGH = receive the value
#define MAX_DISTANCE 400 // cm
};