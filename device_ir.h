#include <Arduino.h>

class IR
{
public:
    void init(void);
    void test(void);
    void get_simple_motor_speed(uint16_t *speed_left_motor, uint16_t *speed_right_motor);

private:
#define PIN_IR_L A2 // Left IR Sensor
#define PIN_IR_M A1 // Middle IR Sensor
#define PIN_IR_R A0 // Right IR Sensor
};