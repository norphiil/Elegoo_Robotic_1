#include <Arduino.h>

enum Direction {
    FORWARDS,
    BACKWARDS,
    RIGHT,
    LEFT,
    FORWARDS_RIGHT,
    FORWARDS_LEFT,
    BACKWARDS_RIGHT,
    BACKWARDS_LEFT
};

class Motor  // TB6612
{
   public:
    // Initialises the pins
    void init(void);

    // One function for all simple movements. Returns false if the direction is not recognised, true otherwise
    void move(Direction direction, uint8_t speed);
    void stop(void);

    // These are the functions responsible for changing pin values, can use them specifically to move non-predefined ways
    void rightMotor(uint8_t direction, uint8_t speed);
    void leftMotor(uint8_t direction, uint8_t speed);

   private:
    // Clamps the input speed between the `MIN_SPEED` and `MAX_SPEED`
    uint8_t normaliseSpeed(uint8_t speed);

    // These functions abstract from the leftMotor and rightMotor functions to provide direction
    void forwards(uint8_t speed);
    void backwards(uint8_t speed);
    void right(uint8_t speed);
    void left(uint8_t speed);
    void forwardsRight(uint8_t speed);
    void forwardsLeft(uint8_t speed);
    void backwardsRight(uint8_t speed);
    void backwardsLeft(uint8_t speed);

#define PIN_MOTOR_A_PWM 5
#define PIN_MOTOR_A_IN 7
#define PIN_MOTOR_B_PWM 6
#define PIN_MOTOR_B_IN 8
#define PIN_MOTOR_STBY 3

#define MAX_SPEED 255
#define MIN_SPEED 30  // Approximately the minimum speed the robot will still move at from our testing

#define MOTOR_FORWARDS 1   // 1 and 0 are the same as HIGH and LOW, these are used in the leftMotor and rightMotor functions
#define MOTOR_BACKWARDS 0  // to indicate the direction
};

class Ultrasonic {
   public:
    void init(void);
    // test to get the distance every second
    void test(void);
    uint16_t get(void);

   private:
    unsigned int microseconds_to_cm(unsigned int microseconds);
#define PIN_TRIG 13       // Pin responsible for the trigger pulse. LOW = prepare for pulse, HIGH = send pulse
#define PIN_ECHO 12       // Pin which gives us the time until the echo pulse was received, HIGH = receive the value
#define MAX_DISTANCE 400  // cm
};

enum IRSensor {
    IR_LEFT,
    IR_MIDDLE,
    IR_RIGHT
};

class IR {
   public:
    // Sets the pins and default threshold (BLACK_ON_WHITE_THRESHOLD)
    void init(void);
    // Sets a custom threshold
    void setThreshold(uint16_t threshold);
    // Logs the values of the 3 sensors
    void test(void);

    // Returns true if the provided sensor senses a line
    bool get(IRSensor sensor);

    // THRESHOLDS
    const uint16_t BLACK_ON_WHITE_THRESHOLD = 400;
    // ...

   private:
    // value at which the robot will recognise a line
    uint16_t threshold;
    const uint8_t PIN_IR_L = A2;  // Left IR Sensor
    const uint8_t PIN_IR_M = A1;  // Middle IR Sensor
    const uint8_t PIN_IR_R = A0;  // Right IR Sensor
};