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

class Ultrasonic
{
public:
    void init(void);
    // test to get the distance every second
    void test(void);
    // void get(uint16_t *get);
    // void calculate_field_of_view(Servo servo);
    uint16_t get_distance(void);

private:
    // unsigned int microseconds_to_cm(unsigned int microseconds);
#define PIN_SERVO 10     // Pin responsible for the servo motor
#define PIN_TRIG 13      // Pin responsible for the trigger pulse. LOW = prepare for pulse, HIGH = send pulse
#define PIN_ECHO 12      // Pin which gives us the time until the echo pulse was received, HIGH = receive the value
#define MAX_DISTANCE 400 // cm
};

class Servo
{
public:
    void init(void);
    void test(void);
    void setAngle(uint8_t new_angle);
    uint8_t find_way(Ultrasonic ultrasonic);

private:
    void setAngleBrute(uint8_t new_angle);
    uint8_t angle;
};

class SimplePID
{
public:
    double compute(double input);

private:
    double kp = 0.1;      // Coefficient proportionnel
    double ki = 0.01;     // Coefficient intégral
    double kd = 0.1;      // Coefficient dérivé
    double dt = 0.10;     // Intervalle de temps entre les échantillons
    double previousError; // Erreur précédente
    double integral;      // Terme intégral
};

class Pos
{
public:
    void init(double x, double y);
    double getX(void);
    double getY(void);
    void set(double x, double y);
    double distanceTo(Pos pos);
    double calculateTargetAngle(Pos pos);
    void toString(void);

private:
    double x,
        y;
};

class GyroAccel
{
public:
    void init(void);
    void getRotation(float *roll, float *pitch, float *yaw);
    // void getPosition(double *x, double *y, double *z);
    bool areAnglesEqual(double angle1, double angle2, double tolerance = 0.01);
    double getAnglesDiff(double angle1, double angle2);
    void testPrint(void);
    void calibrate(void);

private:
    void
    IMU_error(void);
    void getAcceleration(int16_t *ax, int16_t *ay, int16_t *az, bool calibrated = false);
    void getGyroscope(int16_t *gx, int16_t *gy, int16_t *gz, bool calibrated = false);
    void Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat);
    float AcXError, AcYError, AcZError, GyXError, GyYError, GyZError;
    unsigned long last_time_rotation, last_time_acceleration;
    double accel_sensitivity;
    float q[4] = {1.0, 0.0, 0.0, 0.0};
#define gscale ((250. / 32768.0) * (PI / 180.0)) // gyro default 250 LSB per d/s -> rad/s
#define MPU 0b1101000                            // I2C address of the MPU-6050
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
    void goToPoint(Pos current_pos, Pos target_pos, uint8_t speed);
    void testSquare(void);
    GyroAccel getGyroAccel(void);
    void obstacle_stop(void);
    void obstacle_avoidance(void);

private:
    GyroAccel gyroaccel;
    Ultrasonic ultrasonic;
    Servo servo;

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
    void turn(double angle_diff, uint8_t speed);
    void straightLine(Direction direction, uint8_t speed, float initialYaw);

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

class Path
{
public:
    void init(Pos path_list[100], uint8_t number_of_points);
    void run(Motor motor, uint8_t speed, uint8_t number_of_points);

private:
    Pos path_list[100];
};

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