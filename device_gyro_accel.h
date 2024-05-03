#include <Arduino.h>

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