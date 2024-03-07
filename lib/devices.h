#include <Arduino.h>

enum Direction
{
    FORWARDS,
    BACKWARDS,
    RIGHT,
    LEFT,
    FORWARDS_RIGHT,
    FORWARDS_LEFT,
    BACKWARDS_RIGHT,
    BACKWARDS_LEFT
};

class Pos
{
public:
    void init(double x, double y);
    double getX(void);
    double getY(void);
    int16_t distanceTo(Pos pos);
    int16_t calculateTargetAngle(Pos pos);

private:
    double x,
        y;
};

class GyroAccel
{
public:
    void init(void);
    int16_t getAngleX(void);
    int16_t getDistance(int16_t intervalleTemps, int16_t velocity, int16_t distance);
    void test(void);

private:
    int16_t
    getAcceleration(void);
#define PIN_GYRO 2
};

class Motor // TB6612
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

    // One function to make the robot go to specific point
    void goToPoint(Pos current_pos, Pos target_pos, uint8_t speed);

private:
    GyroAccel gyroaccel;

    // Clamps the input speed between the MIN_SPEED and MAX_SPEED
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
    int16_t normalizeAngle(int16_t angle);
    void turn(int16_t angle_diff, uint8_t speed);

#define PIN_MOTOR_A_PWM 5
#define PIN_MOTOR_A_IN 7
#define PIN_MOTOR_B_PWM 6
#define PIN_MOTOR_B_IN 8
#define PIN_MOTOR_STBY 3

#define MAX_SPEED 255
#define MIN_SPEED 30 // Approximately the minimum speed the robot will still move at from our testing

#define MOTOR_FORWARDS 1  // 1 and 0 are the same as HIGH and LOW, these are used in the leftMotor and rightMotor functions
#define MOTOR_BACKWARDS 0 // to indicate the direction
};

class Ultrasonic
{
public:
    void init(void);
    // test to get the distance every second
    void test(void);
    // void get(uint16_t *get);

private:
    // unsigned int microseconds_to_cm(unsigned int microseconds);
#define PIN_TRIG 13      // Pin responsible for the trigger pulse. LOW = prepare for pulse, HIGH = send pulse
#define PIN_ECHO 12      // Pin which gives us the time until the echo pulse was received, HIGH = receive the value
#define MAX_DISTANCE 400 // cm
};

class Path
{
public:
    void init(Pos path_list[10]);
    void run(Motor motor, uint8_t speed);

private:
    Pos path_list[10];
};