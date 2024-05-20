#include "device_gyro_accel.h"
#include "Wire.h"

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

    this->IMU_error();
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
    double difference_plus = ((int)angle1 - (int)angle2 + 360) % 360;
    double difference_minus = ((int)angle2 - (int)angle1 + 360) % 360;
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
        delay(2);
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
        delay(2);
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