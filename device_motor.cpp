#include "device_motor.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////         MOTOR          //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Initialises the motor pins
 */
void Motor::init()
{
    pinMode(PIN_MOTOR_A_PWM, OUTPUT);
    pinMode(PIN_MOTOR_B_PWM, OUTPUT);
    pinMode(PIN_MOTOR_A_IN, OUTPUT);
    pinMode(PIN_MOTOR_B_IN, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // Enable the gyroaccel

    this->ultrasonic = Ultrasonic();
    this->ultrasonic.init();

    this->servo = Servo();
    this->servo.init();

    this->gyroaccel = GyroAccel();
    this->gyroaccel.init();

    this->maze = Maze();
    this->maze.init(6, 6);

    int cell_width = 30; // 30x30 cm

    // this->servo.setAngle(90);
    // int forwardDistance = this->ultrasonic.get_distance();

    // this->servo.setAngle(0);
    // int leftDistance = this->ultrasonic.get_distance();

    // this->servo.setAngle(180);
    // int rightDistance = this->ultrasonic.get_distance();

    // this->servo.setAngle(90);

    // Cell forwardCell = *maze.getCell(ceil(forwardDistance / cell_width), 0);
    // forwardCell.setTopWall(true);
    // maze.setCell(ceil(forwardDistance / cell_width), 0, &forwardCell);

    // Cell leftCell = *maze.getCell(0, ceil(leftDistance / cell_width));
    // leftCell.setLeftWall(true);
    // maze.setCell(0, ceil(leftDistance / cell_width), &leftCell);

    // Cell rightCell = *maze.getCell(0, ceil(rightDistance / cell_width));
    // rightCell.setRightWall(true);
    // maze.setCell(0, ceil(rightDistance / cell_width), &rightCell);

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            Cell currentCell = *maze.getCell(i, j);
            if (i == 0 && j == 3)
            {
                currentCell.setIsStart(true);
            }
            if (i == 3 && j == 4)
            {
                currentCell.setIsGoal(true);
            }
            maze.setCell(i, j, &currentCell);
            if (i == 0)
            {
                maze.setTopWall(i, j, true);
            }
            if (i == 5)
            {
                maze.setBottomWall(i, j, true);
            }
            if (j == 0)
            {
                maze.setLeftWall(i, j, true);
            }
            if (j == 5)
            {
                maze.setRightWall(i, j, true);
            }

            if (i == 1 && j == 1)
            {
                maze.setTopWall(i, j, true);
                maze.setRightWall(i, j, true);
            }
            if (i == 1 && j == 2)
            {
                maze.setTopWall(i, j, true);
                maze.setLeftWall(i, j, true);
            }
            if (i == 1 && j == 3)
            {
                maze.setTopWall(i, j, true);
                maze.setBottomWall(i, j, true);
            }
            if (i == 1 && j == 4)
            {
                maze.setTopWall(i, j, true);
                maze.setBottomWall(i, j, true);
            }
            if (i == 1 && j == 5)
            {
                maze.setBottomWall(i, j, true);
            }
            if (i == 2 && j == 1)
            {
                maze.setBottomWall(i, j, true);
                maze.setRightWall(i, j, true);
            }
            if (i == 2 && j == 2)
            {
                maze.setBottomWall(i, j, true);
                maze.setLeftWall(i, j, true);
            }
            if (i == 2 && j == 3)
            {
                maze.setBottomWall(i, j, true);
                // maze.setRightWall(i, j, true);
            }
            if (i == 3 && j == 1)
            {
                maze.setTopWall(i, j, true);
                // maze.setLeftWall(i, j, true);
            }
            if (i == 3 && j == 2)
            {
                maze.setBottomWall(i, j, true);
                // maze.setLeftWall(i, j, true);
            }
            if (i == 3 && j == 3)
            {
                maze.setBottomWall(i, j, true);
                // maze.setRightWall(i, j, true);
            }
            if (i == 4 && j == 1)
            {
                maze.setTopWall(i, j, true);
                maze.setRightWall(i, j, true);
            }
            if (i == 4 && j == 2)
            {
                maze.setTopWall(i, j, true);
                maze.setLeftWall(i, j, true);
            }
            if (i == 4 && j == 3)
            {
                maze.setTopWall(i, j, true);
                maze.setRightWall(i, j, true);
            }
        }
    }
    maze.floodFillTwice();
    maze.display();
}

/**
 * Returns the GyroAccel object
 */
GyroAccel *Motor::getGyroAccel()
{
    return &this->gyroaccel;
}

/**
 * Clamps the input speed between the MIN_SPEED and MAX_SPEED
 */
uint8_t Motor::normaliseSpeed(uint8_t speed)
{
    speed = speed > MIN_SPEED ? speed : MIN_SPEED;
    speed = speed < MAX_SPEED ? speed : MAX_SPEED;
    return speed;
}

/**
 * Changes the right motor's direction and speed
 */
void Motor::rightMotor(uint8_t direction, uint8_t speed)
{
    digitalWrite(PIN_MOTOR_A_IN, direction);
    analogWrite(PIN_MOTOR_A_PWM, speed);
}

/**
 * Changes the left motor's direction and speed
 */
void Motor::leftMotor(uint8_t direction, uint8_t speed)
{
    digitalWrite(PIN_MOTOR_B_IN, direction);
    analogWrite(PIN_MOTOR_B_PWM, speed);
}

/**
 * One function for all simple movements.
 */
void Motor::move(Direction direction, uint8_t speed_left, uint8_t speed_right)
{
    // Enable both motors
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    // Clamp the speed between [MIN_SPEED, MAX_SPEED]
    speed_left = this->normaliseSpeed(speed_left);
    speed_right = this->normaliseSpeed(speed_right);

    // #ifdef DEBUG_MODE
    //     Serial.print("NORMALISED SPEED=");
    //     Serial.println(speed_left);
    //     Serial.println(speed_right);

    //     Serial.print("DIRECTION=");
    //     Serial.println(direction);
    // #endif

    switch (direction)
    {
    case FORWARDS:
        this->forwards(speed_left, speed_right);
        break;
    case BACKWARDS:
        this->backwards(speed_left, speed_right);
        break;
    case RIGHT:
        this->right(speed_left, speed_right);
        break;
    case LEFT:
        this->left(speed_left, speed_right);
        break;
    case FORWARDS_RIGHT:
        this->forwardsRight(speed_left, speed_right);
        break;
    case FORWARDS_LEFT:
        this->forwardsLeft(speed_left, speed_right);
        break;
    case BACKWARDS_RIGHT:
        this->backwardsRight(speed_left, speed_right);
        break;
    case BACKWARDS_LEFT:
        this->backwardsLeft(speed_left, speed_right);
        break;
    default:
        // In case of an unhandled direction, stop the motors, log the error
        this->stop();
        Serial.println("ERROR: INVALID DIRECTION");
    }
}

void Motor::forwards(uint8_t speed_left, uint8_t speed_right)
{
    this->rightMotor(MOTOR_FORWARDS, speed_right);
    this->leftMotor(MOTOR_FORWARDS, speed_left);
}

void Motor::backwards(uint8_t speed_left, uint8_t speed_right)
{
    this->rightMotor(MOTOR_BACKWARDS, speed_right);
    this->leftMotor(MOTOR_BACKWARDS, speed_left);
}

void Motor::right(uint8_t speed_left, uint8_t speed_right)
{
    this->rightMotor(MOTOR_BACKWARDS, speed_right);
    this->leftMotor(MOTOR_FORWARDS, speed_left);
}

void Motor::left(uint8_t speed_left, uint8_t speed_right)
{
    this->rightMotor(MOTOR_FORWARDS, speed_right);
    this->leftMotor(MOTOR_BACKWARDS, speed_left);
}

void Motor::forwardsRight(uint8_t speed_left, uint8_t speed_right)
{
    this->rightMotor(MOTOR_FORWARDS, speed_right / 2);
    this->leftMotor(MOTOR_FORWARDS, speed_left);
}

void Motor::forwardsLeft(uint8_t speed_left, uint8_t speed_right)
{
    this->rightMotor(MOTOR_FORWARDS, speed_right);
    this->leftMotor(MOTOR_FORWARDS, speed_left / 2);
}

void Motor::backwardsRight(uint8_t speed_left, uint8_t speed_right)
{
    this->rightMotor(MOTOR_BACKWARDS, speed_right / 2);
    this->leftMotor(MOTOR_BACKWARDS, speed_left);
}

void Motor::backwardsLeft(uint8_t speed_left, uint8_t speed_right)
{
    this->rightMotor(MOTOR_BACKWARDS, speed_right);
    this->leftMotor(MOTOR_BACKWARDS, speed_left / 2);
}

void Motor::stop()
{
    analogWrite(PIN_MOTOR_A_PWM, 0);
    analogWrite(PIN_MOTOR_B_PWM, 0);
    digitalWrite(PIN_MOTOR_STBY, LOW);
}

/**
 * Turns the robot by a given angle at a given speed
 * @param angle The angle to turn to (in degrees, 0-360 range)
 * @param speed The speed at which to turn (0-255 range)
 */
void Motor::turn(double angle, uint8_t speed)
{
    if (angle < 0)
    {
        angle += 360;
        angle = fmod(angle, 360);
    }
    float roll, pitch, yaw;
    this->gyroaccel.getRotation(&roll, &pitch, &yaw);
    float current_angle = yaw;
    Direction current_rotate_direction;
    double difference_plus = fmod((angle - current_angle + 360.0), 360.0);
    double difference_minus = fmod((current_angle - angle + 360.0), 360.0);

    double differenceFinale = min(difference_plus, difference_minus);
    if (difference_plus <= difference_minus)
    {
        current_rotate_direction = RIGHT;
    }
    else
    {
        current_rotate_direction = LEFT;
    }

    // Serial.println("Turning");
    // Serial.println(angle);
    // Serial.println(current_angle);
    // Serial.println(differenceFinale);
    // Serial.println(difference_plus);
    // Serial.println(difference_minus);

    uint16_t ind = 0;

    uint8_t new_speed = speed;
    Direction old_rotate_direction = current_rotate_direction;
    float overflow_angle = 0;
    while (!this->gyroaccel.areAnglesEqual(angle, current_angle, 0.5))
    {
        double difference_plus = fmod((angle - current_angle + 360.0), 360.0);
        double difference_minus = fmod((current_angle - angle + 360.0), 360.0);
        double differenceFinale = min(difference_plus, difference_minus);

        // Serial.println("Turning");
        // Serial.println(angle);
        // Serial.println(current_angle);
        // Serial.println(differenceFinale);
        // Serial.println(difference_plus);
        // Serial.println(difference_minus);
        Direction old_current_rotate_direction = current_rotate_direction;

        if (differenceFinale < 90)
        {
            if (difference_plus <= difference_minus)
            {
                current_rotate_direction = RIGHT;
            }
            else
            {
                current_rotate_direction = LEFT;
            }
        }
        if (old_current_rotate_direction != current_rotate_direction)
        {
            this->stop();
            delay(100);
            new_speed = MIN_SPEED * 2;
        }
        else if (differenceFinale < 180 / 2)
        {
            new_speed = (differenceFinale / (180 / 2)) * speed;
            new_speed = new_speed;
        }
        else if (differenceFinale > 180 / 2)
        {
            new_speed = speed;
        }

        this->move(current_rotate_direction, max(MIN_SPEED * 2, new_speed), max(MIN_SPEED * 2, new_speed));
        float roll, pitch, yaw;
        this->gyroaccel.getRotation(&roll, &pitch, &yaw);
        current_angle = yaw;
    }
    this->stop();
}

/**
 * Moves the robot in a straight line at a given speed
 * @param speed The speed at which to move (0-255 range)
 * @param targetDist The target dist of the robot
 */
void Motor::straightLine(uint8_t speed, float targetDist)
{
    unsigned long lastTime = millis();
    // Adjusted PID constants
    const float Kp = 6.2f; // Proportional gain
    const float Ki = 0.5f; // Integral gain
    const float Kd = 1.0f; // Derivative gain

    const int8_t maxSpeed = 50; // Maximum speed

    float errorDist = 0.0f;
    float integralDist = 0.0f;
    float derivativeDist = 0.0f;
    float previousDist = 0.0f;

    const float integralMax = 1.0f;
    const float integralMin = -1.0f;

    this->servo.setAngle(90);
    uint16_t forwardDistance = this->ultrasonic.get_distance();

    while (forwardDistance > 2)
    {
        this->servo.setAngle(180);
        uint16_t leftDistance = this->ultrasonic.get_distance();

        unsigned long currentTime = millis();
        const int delta = currentTime - lastTime;
        lastTime = currentTime;

        float roll, pitch, currentYaw;
        this->gyroaccel.getRotation(&roll, &pitch, &currentYaw);

        errorDist = targetDist - leftDistance;
        Serial.println(errorDist);
        integralDist += errorDist * delta;
        if (integralDist > integralMax)
        {
            integralDist = integralMax;
        }
        else if (integralDist < integralMin)
        {
            integralDist = integralMin;
        }

        derivativeDist = (targetDist - previousDist) / delta;

        previousDist = leftDistance;

        float pidOutput = -Kp * errorDist - Ki * integralDist - Kd * derivativeDist;

        int8_t leftSpeed = speed + pidOutput;
        if (leftSpeed > maxSpeed)
        {
            leftSpeed = maxSpeed;
        }
        else if (leftSpeed < -maxSpeed)
        {
            leftSpeed = -maxSpeed;
        }
        int8_t rightSpeed = speed - pidOutput;
        if (rightSpeed > maxSpeed)
        {
            rightSpeed = maxSpeed;
        }
        else if (rightSpeed < -maxSpeed)
        {
            rightSpeed = -maxSpeed;
        }

        this->move(FORWARDS, leftSpeed, rightSpeed);

        this->servo.setAngle(90);
        uint16_t forwardDistance = this->ultrasonic.get_distance();
    }
    this->stop();
}
