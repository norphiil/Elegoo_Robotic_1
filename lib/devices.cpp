#include "devices.h"
#include "Wire.h"

// Change to 0 or comment this line out to switch off debug mode and hide Serial prints
#define DEBUG_MODE 1

//////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////         MOTOR          //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Motor::init()
{
    pinMode(PIN_MOTOR_A_PWM, OUTPUT);
    pinMode(PIN_MOTOR_B_PWM, OUTPUT);
    pinMode(PIN_MOTOR_A_IN, OUTPUT);
    pinMode(PIN_MOTOR_B_IN, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);

    // Enable the gyroaccel
    this->gyroaccel = GyroAccel();
    this->gyroaccel.init();
}

uint8_t Motor::normaliseSpeed(uint8_t speed)
{
    speed = speed > MIN_SPEED ? speed : MIN_SPEED;
    speed = speed < MAX_SPEED ? speed : MAX_SPEED;
    return speed;
}

void Motor::rightMotor(uint8_t direction, uint8_t speed)
{
    Serial.print("RIGHT MOTOR MOVING: ");
    Serial.println(direction);
    digitalWrite(PIN_MOTOR_A_IN, direction);
    analogWrite(PIN_MOTOR_A_PWM, speed);
}

void Motor::leftMotor(uint8_t direction, uint8_t speed)
{
    digitalWrite(PIN_MOTOR_B_IN, direction);
    analogWrite(PIN_MOTOR_B_PWM, speed);
}

void Motor::move(Direction direction, uint8_t speed)
{
    // Enable both motors
    digitalWrite(PIN_MOTOR_STBY, HIGH);

    // Clamp the speed between [MIN_SPEED, MAX_SPEED]
    speed = this->normaliseSpeed(speed);

#ifdef DEBUG_MODE
    Serial.print("NORMALISED SPEED=");
    Serial.println(speed);

    Serial.print("DIRECTION=");
    Serial.println(direction);
#endif

    switch (direction)
    {
    case FORWARDS:
        this->forwards(speed);
        break;
    case BACKWARDS:
        this->backwards(speed);
        break;
    case RIGHT:
        this->right(speed);
        break;
    case LEFT:
        this->left(speed);
        break;
    case FORWARDS_RIGHT:
        this->forwardsRight(speed);
        break;
    case FORWARDS_LEFT:
        this->forwardsLeft(speed);
        break;
    case BACKWARDS_RIGHT:
        this->backwardsRight(speed);
        break;
    case BACKWARDS_LEFT:
        this->backwardsLeft(speed);
        break;
    default:
        // In case of an unhandled direction, stop the motors, log the error
        this->stop();
        Serial.println("ERROR: INVALID DIRECTION");
    }
}

void Motor::forwards(uint8_t speed)
{
    Serial.print("moving forwards");
    this->rightMotor(MOTOR_FORWARDS, speed);
    this->leftMotor(MOTOR_FORWARDS, speed);
}

void Motor::backwards(uint8_t speed)
{
    this->rightMotor(MOTOR_BACKWARDS, speed);
    this->leftMotor(MOTOR_BACKWARDS, speed);
}

void Motor::right(uint8_t speed)
{
    this->rightMotor(MOTOR_BACKWARDS, speed);
    this->leftMotor(MOTOR_FORWARDS, speed);
}

void Motor::left(uint8_t speed)
{
    this->rightMotor(MOTOR_FORWARDS, speed);
    this->leftMotor(MOTOR_BACKWARDS, speed);
}

void Motor::forwardsRight(uint8_t speed)
{
    this->rightMotor(MOTOR_FORWARDS, speed / 2);
    this->leftMotor(MOTOR_FORWARDS, speed);
}

void Motor::forwardsLeft(uint8_t speed)
{
    this->rightMotor(MOTOR_FORWARDS, speed);
    this->leftMotor(MOTOR_FORWARDS, speed / 2);
}

void Motor::backwardsRight(uint8_t speed)
{
    this->rightMotor(MOTOR_BACKWARDS, speed / 2);
    this->leftMotor(MOTOR_BACKWARDS, speed);
}

void Motor::backwardsLeft(uint8_t speed)
{
    this->rightMotor(MOTOR_BACKWARDS, speed);
    this->leftMotor(MOTOR_BACKWARDS, speed / 2);
}

void Motor::stop()
{
    analogWrite(PIN_MOTOR_A_PWM, 0);
    analogWrite(PIN_MOTOR_B_PWM, 0);
    digitalWrite(PIN_MOTOR_STBY, LOW);
}

void Motor::turn(int16_t angle, uint8_t speed)
{
    int16_t current_angle = this->gyroaccel.getAngleX();
    Direction current_rotate_direction;
    if (angle > current_angle && angle - current_angle < 180)
    {
        current_rotate_direction = RIGHT;
    }
    else
    {
        current_rotate_direction = LEFT;
    }

    this->move(current_rotate_direction, speed);

    while (current_angle != angle)
    {
        Direction old_current_rotate_direction = current_rotate_direction;
        delay(100);
        if (angle > current_angle && angle - current_angle < 180)
        {
            current_rotate_direction = RIGHT;
        }
        else
        {
            current_rotate_direction = LEFT;
        }
        if (current_rotate_direction != old_current_rotate_direction)
        {
            this->stop();
            delay(100);
            speed = speed / 2;
            this->move(current_rotate_direction, speed);
        }
        current_angle = gyroaccel.getAngleX();
    }
}

int16_t Motor::normalizeAngle(int16_t angle)
{
    angle = fmod(angle + 180.0, 360.0);
    if (angle > 180.0)
    {
        angle -= 360.0;
    }
    return angle;
}

// int16_t Motor::getEulerAngles(float Yaw)
// {
//     int16_t gz = this->gyroaccel.getAngleZ();
//     float gyroz = -(gz - gzo) / 131.0 * dt; // z轴角速度
//     if (fabs(gyroz) < 0.05)
//     {
//         gyroz = 0.00;
//     }
//     agz += gyroz; // z轴角速度积分
//     *Yaw = agz;
// }

void Motor::straightLine(Direction direction, uint8_t speed)
{
    static uint8_t UpperLimit = 255;
    static float Yaw;
    static float yaw_So = 0;
    static uint8_t en = 110;
    static unsigned long is_time;
    if (millis() - is_time > 10)
    {
        this->stop();
        Yaw = this->gyroaccel.getAngleZ();

        is_time = millis();
    }

    int R = (Yaw - yaw_So) * 10 + speed;
    if (R > UpperLimit)
    {
        R = UpperLimit;
    }
    else if (R < 10)
    {
        R = 10;
    }
    int L = (yaw_So - Yaw) * 10 + speed;
    if (L > UpperLimit)
    {
        L = UpperLimit;
    }
    else if (L < 10)
    {
        L = 10;
    }
    if (direction == FORWARDS)
    {
        this->rightMotor(MOTOR_FORWARDS, R);
        this->leftMotor(MOTOR_FORWARDS, L);
    }
    else if (direction == BACKWARDS)
    {
        this->rightMotor(MOTOR_BACKWARDS, L);
        this->leftMotor(MOTOR_BACKWARDS, R);
    }
}

void Motor::goToPoint(Pos current_pos, Pos target_pos, uint8_t speed)
{
    int16_t distance = current_pos.distanceTo(target_pos);
    int16_t current_velocity = 0;
    int16_t current_distance = 0;

    bool go_forward = false;
    uint8_t interval_time = 100;
    uint16_t start_time = millis();

    this->turn(current_pos.calculateTargetAngle(target_pos), speed);

    // Check the unit for distance in cm or mm or m ?
    while (distance > 1)
    {
        delay(interval_time);
        uint16_t end_time = millis();
        uint16_t time_between = end_time - start_time;
        int16_t result[2] = {};
        result[2] = this->gyroaccel.getDistance(time_between, current_velocity, current_distance);
        int16_t current_velocity = result[0];
        int16_t current_distance = result[1];
        distance = distance - current_distance;

        this->straightLine(FORWARDS, speed);

        start_time = end_time;
    }
    this->stop();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////      ULTRASONIC     //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Ultrasonic::init()
{
    pinMode(PIN_ECHO, INPUT);
    pinMode(PIN_TRIG, OUTPUT);
}

void Ultrasonic::test()
{
    digitalWrite(PIN_TRIG, LOW); // Preparing to send the ultrasonic pulse
    delayMicroseconds(2);        // Waiting for above pin to change

    digitalWrite(PIN_TRIG, HIGH); // Send the pulse
    delayMicroseconds(10);        // Wait for it to send

    digitalWrite(PIN_TRIG, LOW); // Go back to prepare mode

    /* Takes the time in microseconds for the sound to travel TO AND FROM the object from the echo pin.
     * Divided by 58 to convert to cm. 29 is the approximate time taken for sound to travel 1cm. The sound
     * travels both TO and FROM the object, so twice 29 is 58.
     */
    unsigned int distanceCm = ((unsigned int)pulseIn(PIN_ECHO, HIGH) / 58);

    Serial.print("ultrasonic_sensor_test=");
    Serial.print(distanceCm);
    Serial.println("cm");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////      GYROSCOPE AND ACCELEROMETER      /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void GyroAccel::init()
{
    Wire.begin();
    Wire.beginTransmission(PIN_GYRO);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

void GyroAccel::readGyroAccel()
{
    Wire.beginTransmission(PIN_GYRO);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(PIN_GYRO, 14, true);

    this->AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    this->AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    this->AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    this->Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    this->GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    this->GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    this->GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

int16_t GyroAccel::getTemperature()
{
    return this->Tmp;
}

int16_t GyroAccel::getAngleX()
{
    return this->GyX;
}

int16_t GyroAccel::getAngleY()
{
    return this->GyY;
}

int16_t GyroAccel::getAngleZ()
{
    return this->GyZ;
}

int16_t GyroAccel::getAcceleration()
{
    int16_t acceleration[3] = {this->AcX, this->AcY, this->AcZ};
    return acceleration;
}

int16_t
GyroAccel::getDistance(int16_t intervaleTemps, int16_t velocity, int16_t distance)
{
    int16_t acceleration[3] = {};
    acceleration[3] = this->getAcceleration();
    int16_t AcX = acceleration[0];
    int16_t AcY = acceleration[1];
    int16_t AcZ = acceleration[2];

    double nouvelleAcceleration = sqrt(AcX * AcX + AcY * AcY + AcZ * AcZ);

    velocity += nouvelleAcceleration * intervaleTemps;

    distance += velocity * intervaleTemps;
    int16_t result[2] = {velocity, distance};
    return result;
}

void GyroAccel::test()
{
    delay(1000);
    this->readGyroAccel();
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.println("");
    Serial.print("accelerometer_x=");
    Serial.println(this->AcX);
    Serial.print("accelerometer_y=");
    Serial.println(this->AcY);
    Serial.print("accelerometer_z=");
    Serial.println(this->AcZ);
    Serial.print("temperature=");
    Serial.println(this->Tmp / 340.00 + 36.53);
    Serial.print("gyroscope_x=");
    Serial.println(this->GyX);
    Serial.print("gyroscope_y=");
    Serial.println(this->GyY);
    Serial.print("gyroscope_z=");
    Serial.println(this->GyZ);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////      TARGET      //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void Pos::init(double x, double y)
{
    this->x = x;
    this->y = y;
}

double Pos::getX()
{
    return this->x;
}

double Pos::getY()
{
    return this->y;
}

int16_t Pos::distanceTo(Pos pos)
{
    double x_diff = pos.getX() - this->x;
    double y_diff = pos.getY() - this->y;
    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

int16_t Pos::calculateTargetAngle(Pos pos)
{
    double x_diff = pos.getX() - this->x;
    double y_diff = pos.getY() - this->y;
    return atan2(y_diff, x_diff) * 180 / PI;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////      PATH      ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

void Path::init(Pos path_list[10])
{
    for (int i = 0; i < 10; i++)
    {
        this->path_list[i] = *(path_list + i);
    }
}

void Path::run(Motor motor, uint8_t speed)
{
    Pos current_pos = Pos();
    current_pos.init(0, 0);
    for (int i = 0; i < 10; i++)
    {
        motor.goToPoint(current_pos, this->path_list[i], speed);
        current_pos = this->path_list[i];
    }

    Serial.println("Path completed!");
}
