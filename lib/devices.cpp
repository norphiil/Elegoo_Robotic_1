#include "devices.h"
#include "Wire.h"

// Change to 0 or comment this line out to switch off debug mode and hide Serial prints
#define DEBUG_MODE 1

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
    this->gyroaccel = GyroAccel();
    this->gyroaccel.init();
}

/**
 * Returns the GyroAccel object
 */
GyroAccel Motor::getGyroAccel()
{
    return this->gyroaccel;
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

    Serial.println("Turning");
    Serial.println(angle);
    Serial.println(current_angle);
    Serial.println(differenceFinale);
    Serial.println(difference_plus);
    Serial.println(difference_minus);

    uint16_t ind = 0;

    uint8_t new_speed = speed;
    Direction old_rotate_direction = current_rotate_direction;
    const double decelerationFactor = 0.5;
    float overflow_angle = 0;
    while (!this->gyroaccel.areAnglesEqual(angle, current_angle, 0.06))
    {
        double difference_plus = fmod((angle - current_angle + 360.0), 360.0);
        double difference_minus = fmod((current_angle - angle + 360.0), 360.0);
        double differenceFinale = min(difference_plus, difference_minus);

        Serial.println("Turning");
        Serial.println(angle);
        Serial.println(current_angle);
        Serial.println(differenceFinale);
        Serial.println(difference_plus);
        Serial.println(difference_minus);
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
 * @param direction The direction to move in (FORWARDS or BACKWARDS)
 * @param speed The speed at which to move (0-255 range)
 * @param initialYaw The initial yaw angle of the robot
 */
void Motor::straightLine(Direction direction, uint8_t speed, float initialYaw)
{
    const float targetAngle = 0.0;
    const float angleTolerance = 2.0;
    const float correctionFactor = 0.05;

    float roll, pitch, currentYaw;
    this->gyroaccel.getRotation(&roll, &pitch, &currentYaw);

    float angleDifference = this->gyroaccel.getAnglesDiff(currentYaw, initialYaw);

    int speedLeft = speed;
    int speedRight = speed;

    if (angleDifference > angleTolerance)
    {
        speedLeft = static_cast<int>(speed * (1.0 - correctionFactor));
    }
    else if (angleDifference < -angleTolerance)
    {
        speedRight = static_cast<int>(speed * (1.0 - correctionFactor));
    }

    this->move(direction, speedLeft, speedRight);
}

/**
 * Moves the robot to a given point at a given speed
 * @param current_pos The current position of the robot
 * @param target_pos The target position of the robot
 * @param speed The speed at which to move
 */
void Motor::goToPoint(Pos current_pos, Pos target_pos, uint8_t speed)
{
    double distance = current_pos.distanceTo(target_pos);
    Serial.println("Distance:");
    Serial.println(distance);
    float roll, pitch, yaw;
    this->gyroaccel.getRotation(&roll, &pitch, &yaw);
    double angle = current_pos.calculateTargetAngle(target_pos);
    this->turn(angle, speed);
    // Serial.println("Turning done");
    delay(1000);
    long start_time = millis();
    while (distance > 0.1)
    {
        this->straightLine(FORWARDS, speed, yaw);
        if (millis() - start_time > 1000 * distance)
        {
            distance = 0.01;
        }
        // double x, y, z;
        // this->gyroaccel.getPosition(&x, &y, &z);
        // current_pos.set(x, y);
        // distance = current_pos.distanceTo(target_pos);
        // Serial.println("Distance:");
        // Serial.println(distance);
        // Serial.println("Target:");
        // Serial.println(target_pos.getX());
        // Serial.println(target_pos.getY());
        // Serial.println("Current:");
        // Serial.println(current_pos.getX());
        // Serial.println(current_pos.getY());
        delay(10);
    }
    this->stop();
}

/**
 * Moves the robot along a given path
 */
void Motor::testSquare()
{
    // Pos pos1;
    // pos1.init(0, 1);
    // Pos pos2;
    // pos2.init(0, -1);
    // Pos pos3;
    // pos3.init(-1, 0);
    // Pos pos4;
    // pos4.init(1, 0);

    Pos pos1;
    pos1.init(0, 1);
    Pos pos2;
    pos2.init(1, 0);
    Pos pos3;
    pos3.init(0, -1);
    Pos pos4;
    pos4.init(-2, 0);
    Pos pos5;
    pos5.init(1, 0);

    uint8_t pos_number = 5;
    Pos path_list[pos_number] = {pos1, pos2, pos3, pos4, pos5};

    Path path;
    path.init(path_list, pos_number);
    path.run(*this, 100, pos_number);

    // double x, y, z;
    // this->gyroaccel.getPosition(&x, &y, &z);
    // Pos current_pos = Pos();
    // current_pos.init(0, 0);
    // double angle_d = 0;
    // for (int i = 0; i < pos_number; i++)
    // {
    //     Pos target_pos = path_list[i];
    //     Serial.println("Moving to next point");
    //     double angle = current_pos.calculateTargetAngle(target_pos);
    //     Serial.println("angle: ");
    //     Serial.println(angle);
    //     // current_pos = target_pos;
    //     this->turn(angle, 100);
    //     delay(1000);
    // }
    // delay(5000);
}

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

/**
 * Sends a pulse to the ultrasonic sensor and measures the time it takes for the echo to return
 * Print the distance to the nearest object in centimeters
 */
void Ultrasonic::test()
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
    uint16_t distanceCm = duration * 0.034 / 2;
    Serial.print("ultrasonic_sensor_test=");
    Serial.print(distanceCm);
    Serial.println("cm");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////      SERVO     ///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Servo::init()
{
    // Servo motor
    pinMode(PIN_SERVO, OUTPUT); // Set servo pin as output
    this->setAngle(90);
}

/**
 * Turns the servo motor to a given angle (0-180)
 */
void Servo::setAngle(uint8_t angle)
{
    // Convert degrees to pulse width
    int pulseWidth = map(angle, 0, 180, 500, 2400);
    // Set servo position
    digitalWrite(PIN_SERVO, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(PIN_SERVO, LOW);
}

void Servo::test()
{
    for (int i = 90; i < 180; i++)
    {
        this->setAngle(i);
        delay(15);
    }
    for (int i = 180; i > 0; i--)
    {
        this->setAngle(i);
        delay(15);
    }
    for (int i = 0; i < 90; i++)
    {
        this->setAngle(i);
        delay(15);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////      GYROSCOPE AND ACCELEROMETER      /////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialises the gyro and accelerometer
 */
void GyroAccel::init()
{
    Wire.begin();                // Initialize comunication
    Wire.beginTransmission(MPU); // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);            // Talk to the register 6B
    Wire.write(0b00000000);      // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);  // end the transmission

    delay(20);

    int accel_2g_address = 0x00;
    int accel_4g_address = 0x01;
    int accel_8g_address = 0x02;
    int accel_16g_address = 0x03;
    double accel_2g_sensor = 16384.0;
    double accel_4g_sensor = 8192.0;
    double accel_8g_sensor = 4096.0;
    double accel_16g_sensor = 2048.0;
    accel_sensitivity = accel_16g_sensor;

    // // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
    // Wire.beginTransmission(MPU);
    // Wire.write(0x1B); // Talk to the GYRO_CONFIG register (1B hex)
    // Wire.write(0b00000000);
    // Wire.endTransmission();
    // // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
    // Wire.beginTransmission(MPU);
    // Wire.write(0x1C); // Talk to the ACCEL_CONFIG register (1C hex)
    // Wire.write(accel_16g_address);
    // Wire.endTransmission();
    delay(20);

    this->IMU_error();
    delay(20);
    this->last_time_rotation = micros();
}

void GyroAccel::calibrate()
{
    Serial.println("Calibrate...");

    float roll, pitch, yaw;
    this->getRotation(&roll, &pitch, &yaw);
    float current_angle = yaw;
    Serial.println(current_angle);
    bool calibration_done = false;
    while (!calibration_done)
    {
        this->getRotation(&roll, &pitch, &yaw);
        float current_angle = yaw;
        Serial.println(current_angle);
        if (this->areAnglesEqual(0, current_angle, 5))
        {
            Serial.println("Calibration done");
            calibration_done = true;
        }
        else
        {
            Serial.println("Calibration not done");
        }
        delay(100);
    }
}

/**
 * Returns the difference between two angles
 * @param angle1 First angle (in degrees from 0 to 360)
 * @param angle2 Second angle (in degrees from 0 to 360)
 * @return The difference between the two angles
 */
double GyroAccel::getAnglesDiff(double angle1, double angle2)
{
    double difference_plus = fmod((angle1 - angle2 + 360.0), 360.0);
    double difference_minus = fmod((angle2 - angle1 + 360.0), 360.0);
    double differenceFinale = min(difference_plus, difference_minus);
    return differenceFinale;
}

/**
 * Compares two angles and returns true if their difference is less than or equal to a given tolerance.
 * @param angle1 First angle
 * @param angle2 Second angle
 * @param tolerance Comparison tolerance
 * @return True if angles are equal, false otherwise
 *
 */
bool GyroAccel::areAnglesEqual(double angle1, double angle2, double tolerance = 0.01)
{
    double differenceFinale = this->getAnglesDiff(angle1, angle2);

    // Comparaison avec la tolérance
    return abs(differenceFinale) <= tolerance;
}

/**
 * Calibrates the IMU by calculating the error values for the accelerometer and gyroscope
 */
void GyroAccel::IMU_error()
{
    Serial.println("Calibrating IMU...");
    long axError = 0, ayError = 0, azError = 0;
    uint16_t c = 0;
    float nb = 500.0;
    while (c < (int)nb)
    {
        int16_t ax, ay, az;
        this->getAcceleration(&ax, &ay, &az);

        // Sum all readings
        axError += (ax);
        ayError += (ay);
        azError += (az);
        c++;
        delay(1);
        // delay(2000 / nb);
    }
    this->AcXError = ((float)axError) / (float)nb;
    this->AcYError = ((float)ayError) / (float)nb;
    this->AcZError = ((float)azError) / (float)nb;

    long gxError = 0, gyError = 0, gzError = 0;
    c = 0;
    while (c < (int)nb)
    {
        int16_t gx, gy, gz;
        this->getGyroscope(&gx, &gy, &gz);

        // Sum all readings
        gxError += (gx);
        gyError += (gy);
        gzError += (gz);

        c++;
        // delay(2000 / nb);
        delay(1);
    }
    this->GyXError = ((float)gxError) / (float)nb;
    this->GyYError = ((float)gyError) / (float)nb;
    this->GyZError = ((float)gzError) / (float)nb;

    // Serial.println("IMU Error:");
    // Serial.print("AcXError=");
    // Serial.print(axError);
    // Serial.print("| AcYError=");
    // Serial.print(ayError);
    // Serial.print("| AcZError=");
    // Serial.print(azError);
    // Serial.print("| GyXError=");
    // Serial.print(gxError);
    // Serial.print("| GyYError=");
    // Serial.print(gyError);
    // Serial.print("| GyZError=");
    // Serial.println(gzError);

    Serial.println("IMU Error:");
    Serial.print("AcXError=");
    Serial.print(this->AcXError);
    Serial.print("| AcYError=");
    Serial.print(this->AcYError);
    Serial.print("| AcZError=");
    Serial.print(this->AcZError);
    Serial.print("| GyXError=");
    Serial.print(this->GyXError);
    Serial.print("| GyYError=");
    Serial.print(this->GyYError);
    Serial.print("| GyZError=");
    Serial.println(this->GyZError);
}

/**
 * Reads the gyroscope values from the MPU6050
 * @param gX Pointer to the variable to store the gyroscope X value
 * @param gY Pointer to the variable to store the gyroscope Y value
 * @param gZ Pointer to the variable to store the gyroscope Z value
 * @param calibrated Whether to apply the error correction or not
 */
void GyroAccel::getGyroscope(int16_t *gX, int16_t *gY, int16_t *gZ, bool calibrated = false)
{
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);
    *gX = ((Wire.read() << 8 | Wire.read()));
    *gY = ((Wire.read() << 8 | Wire.read()));
    *gZ = ((Wire.read() << 8 | Wire.read()));

    if (calibrated)
    {
        *gX = (*gX - this->GyXError);
        *gY = (*gY - this->GyYError);
        *gZ = (*gZ - this->GyZError);
    }
}

/**
 * Reads the accelerometer values from the MPU6050
 * @param aX Pointer to the variable to store the accelerometer X value
 * @param aY Pointer to the variable to store the accelerometer Y value
 * @param aZ Pointer to the variable to store the accelerometer Z value
 * @param calibrated Whether to apply the error correction or not
 */
void GyroAccel::getAcceleration(int16_t *aX, int16_t *aY, int16_t *aZ, bool calibrated = false)
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);
    *aX = ((Wire.read() << 8 | Wire.read()));
    *aY = ((Wire.read() << 8 | Wire.read()));
    *aZ = ((Wire.read() << 8 | Wire.read()));
    if (calibrated)
    {
        *aX = (*aX - this->AcXError);
        *aY = (*aY - this->AcYError);
        *aZ = (*aZ - this->AcZError);
    }
}

/**
 * Reads the rotation values from the MPU6050
 * @param roll Pointer to the variable to store the roll value
 * @param pitch Pointer to the variable to store the pitch value
 * @param yaw Pointer to the variable to store the yaw value
 */
void GyroAccel::getRotation(float *roll, float *pitch, float *yaw)
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t Tmp; // temperature
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 14); // request a total of 14 registers
    int t = Wire.read() << 8;
    ax = t | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    t = Wire.read() << 8;
    ay = t | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    t = Wire.read() << 8;
    az = t | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    t = Wire.read() << 8;
    Tmp = t | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    t = Wire.read() << 8;
    gx = t | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    t = Wire.read() << 8;
    gy = t | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    t = Wire.read() << 8;
    gz = t | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    float Axyz[3];
    float Gxyz[3];

    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;
    float A_cal[6] = {265.0, -80.0, -700.0, 0.994, 1.000, 1.014};
    // float A_cal[6] = {0.0, 0.0, 0.0, 1.000, 1.000, 1.000}; // 0..2 offset xyz, 3..5 scale xyz
    // apply offsets and scale factors from Magneto
    for (int i = 0; i < 3; i++)
        Axyz[i] = (Axyz[i] - A_cal[i]) * A_cal[i + 3];

    Gxyz[0] = ((float)gx - this->GyXError) * gscale; // 250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - this->GyYError) * gscale;
    Gxyz[2] = ((float)gz - this->GyZError) * gscale;

    unsigned long now = micros();
    float deltat = (now - this->last_time_rotation) * 1.0e-6; // seconds since last update
    this->last_time_rotation = now;

    this->Mahony_update(Axyz[0], Axyz[1], Axyz[2], Gxyz[0], Gxyz[1], Gxyz[2], deltat);

    float roll_res = atan2((this->q[0] * this->q[1] + this->q[2] * this->q[3]), 0.5 - (this->q[1] * this->q[1] + this->q[2] * this->q[2]));
    float pitch_res = asin(2.0 * (this->q[0] * this->q[2] - this->q[1] * this->q[3]));

    float yaw_res = -atan2((this->q[1] * this->q[2] + this->q[0] * this->q[3]), 0.5 - (this->q[2] * this->q[2] + this->q[3] * this->q[3]));
    // to degrees
    yaw_res *= 180.0 / PI;
    if (yaw_res < 0)
        yaw_res += 360.0; // compass circle
    // correct for local magnetic declination here
    pitch_res *= 180.0 / PI;
    roll_res *= 180.0 / PI;

    *roll = roll_res;
    *pitch = pitch_res;
    *yaw = yaw_res;
}

/**
 * Mahony filter update
 */
void GyroAccel::Mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat)
{
    float Kp = 30.0;
    float Ki = 0.0;
    float recipNorm;
    float vx, vy, vz;
    float ex, ey, ez; // error terms
    float qa, qb, qc;
    static float ix = 0.0, iy = 0.0, iz = 0.0; // integral feedback terms
    float tmp;

    tmp = ax * ax + ay * ay + az * az;

    // ignore accelerometer if false
    if (tmp > 0.0)
    {

        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recipNorm = 1.0 / sqrt(tmp);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = this->q[1] * this->q[3] - this->q[0] * this->q[2];
        vy = this->q[0] * this->q[1] + this->q[2] * this->q[3];
        vz = this->q[0] * this->q[0] - 0.5f + this->q[3] * this->q[3];

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // Compute and apply to gyro term the integral feedback, if enabled
        if (Ki > 0.0f)
        {
            ix += Ki * ex * deltat; // integral error scaled by Ki
            iy += Ki * ey * deltat;
            iz += Ki * ez * deltat;
            gx += ix; // apply integral feedback
            gy += iy;
            gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
    }

    deltat = 0.5 * deltat;
    gx *= deltat; // pre-multiply common factors
    gy *= deltat;
    gz *= deltat;
    qa = this->q[0];
    qb = this->q[1];
    qc = this->q[2];

    // add qmult*delta_t to current orientation
    this->q[0] += (-qb * gx - qc * gy - this->q[3] * gz);
    this->q[1] += (qa * gx + qc * gz - this->q[3] * gy);
    this->q[2] += (qa * gy - qb * gz + this->q[3] * gx);
    this->q[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = 1.0 / sqrt(this->q[0] * this->q[0] + this->q[1] * this->q[1] + this->q[2] * this->q[2] + this->q[3] * this->q[3]);
    this->q[0] = this->q[0] * recipNorm;
    this->q[1] = this->q[1] * recipNorm;
    this->q[2] = this->q[2] * recipNorm;
    this->q[3] = this->q[3] * recipNorm;
}

/**
 * Not working
 */
// void GyroAccel::calculateCurrentDistance()
// {
//     double new_time = millis();
//     double dt = (new_time - this->last_time_acceleration) / 1000.0;
//     this->last_time_acceleration = new_time;

//     int16_t ax, ay, az;
//     this->getAcceleration(&ax, &ay, &az, true);
//     if (abs(ax) > 0.15)
//     {
//         this->current_distance_x += ax * dt;
//     }
//     if (abs(ay) > 0.15)
//     {
//         this->current_distance_y += ay * dt;
//     }
//     if (abs(az) > 0.15)
//     {
//         this->current_distance_z += az * dt;
//     }
// }

// void GyroAccel::getPosition(double *x, double *y, double *z)
// {
//     this->calculateCurrentDistance();
//     *x = this->current_distance_x;
//     *y = this->current_distance_y;
//     *z = this->current_distance_z;
// }

/**
 * Some print tests of the gyro and accelerometer
 */
void GyroAccel::testPrint()
{
    int16_t gx, gy, gz;
    this->getGyroscope(&gx, &gy, &gz, true);

    int16_t ax, ay, az;
    this->getAcceleration(&ax, &ay, &az, true);
    delay(100);
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
    Serial.println("Gyroscope:");
    Serial.print("X=");
    Serial.print(gx);
    Serial.print("| Y=");
    Serial.print(gy);
    Serial.print("| Z=");
    Serial.println(gz);
    Serial.print("Acceleration:");
    Serial.print("X=");
    Serial.print(ax);
    Serial.print("| Y=");
    Serial.print(ay);
    Serial.print("| Z=");
    Serial.println(az);
    // Serial.print("Position:");
    // Serial.print("X=");
    // Serial.print(x);
    // Serial.print("| Y=");
    // Serial.print(y);
    // Serial.print("| Z=");
    // Serial.println(z);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////      TARGET      //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialises the position
 */
void Pos::init(double x, double y)
{
    this->x = x;
    this->y = y;
}

/**
 * Returns the X coordinate of the position
 * @return The X coordinate
 */
double Pos::getX()
{
    return this->x;
}

/**
 * Returns the Y coordinate of the position
 * @return The Y coordinate
 */
double Pos::getY()
{
    return this->y;
}

/**
 * Sets the X and Y coordinates of the position
 * @param x The new X coordinate
 * @param y The new Y coordinate
 */
void Pos::set(double x, double y)
{
    this->x = x;
    this->y = y;
}

/**
 * Returns the distance to another position
 * @param pos The other position
 * @return The distance to the other position
 */
double Pos::distanceTo(Pos pos)
{
    double x_diff = pos.getX() - this->x;
    double y_diff = pos.getY() - this->y;
    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

/**
 * Prints the position to the serial monitor
 */
void Pos::toString()
{
    Serial.print("X=");
    Serial.print(this->x);
    Serial.print("| Y=");
    Serial.println(this->y);
}

/**
 * Calculates the angle to another position
 * @param pos The other position
 * @return The angle to the other position
 */
double Pos::calculateTargetAngle(Pos pos)
{
    double x_diff = pos.getX() - this->x;
    double y_diff = pos.getY() - this->y;
    return atan2(y_diff, x_diff) * 180 / PI;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////      PATH      ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialises the path
 * @param path_list The list of positions in the path
 */
void Path::init(Pos path_list[100], uint8_t number_of_points)
{
    for (int i = 0; i < number_of_points; i++)
    {
        this->path_list[i] = *(path_list + i);
    }
}

/**
 * Runs the path
 * @param motor The motor object to use
 * @param speed The speed at which to move
 */
void Path::run(Motor motor, uint8_t speed, uint8_t number_of_points)
{
    for (int i = 0; i < number_of_points; i++)
    {
        if (this->path_list[i].getX() != this->path_list[i].getX() || this->path_list[i].getY() != this->path_list[i].getY())
        {
            break;
        }
        Pos current_pos = Pos();
        current_pos.init(0, 0);

        motor.goToPoint(current_pos, this->path_list[i], speed);
    }

    Serial.println("Path completed!");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////      SIMPLEPID      ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

double SimplePID::compute(double input)
{
    double error = input;
    this->integral += error * this->dt;
    double derivative = (error - this->previousError) / this->dt;
    double output = this->kp * error + this->ki * this->integral + this->kd * derivative;
    this->previousError = error;
    return output;
}
