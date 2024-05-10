#include <Arduino.h>
#include "device_servo.h"
#include "device_ir.h"
#include "device_ultrasonic.h"
#include "device_gyro_accel.h"
#include "device_maze.h"

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

class Motor // TB6612
{
public:
    // Initialises the pins
    void init(void);

    // One function for all simple movements. Returns false if the direction is not recognised, true otherwise
    void move(Direction direction, uint8_t speed_left, uint8_t speed_right);
    void stop(void);

    // These are the functions responsible for changing pin values, can use them specifically to move non-predefined ways
    void rightMotor(uint8_t direction, uint8_t speed);
    void leftMotor(uint8_t direction, uint8_t speed);

    // One function to make the robot go to specific point
    GyroAccel *getGyroAccel(void);
    void turn(double angle_diff, uint8_t speed);
    void straightLine(uint8_t speed, float targetYaw);
    Ultrasonic ultrasonic;
    Servo servo;
    GyroAccel gyroaccel;
    Maze maze;

private:
    // Clamps the input speed between the MIN_SPEED and MAX_SPEED
    uint8_t normaliseSpeed(uint8_t speed);

    // These functions abstract from the leftMotor and rightMotor functions to provide direction
    void forwards(uint8_t speed_left, uint8_t speed_right);
    void backwards(uint8_t speed_left, uint8_t speed_right);
    void right(uint8_t speed_left, uint8_t speed_right);
    void left(uint8_t speed_left, uint8_t speed_right);
    void forwardsRight(uint8_t speed_left, uint8_t speed_right);
    void forwardsLeft(uint8_t speed_left, uint8_t speed_right);
    void backwardsRight(uint8_t speed_left, uint8_t speed_right);
    void backwardsLeft(uint8_t speed_left, uint8_t speed_right);

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