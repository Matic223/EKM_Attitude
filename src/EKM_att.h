#ifndef EKM_ATT
#define EKM_ATT
 #include <Arduino.h>

 struct kalmanICMData
{
    float roll;   // deg
    float pitch;  // deg
    float yaw;    // deg, gyro-integrated, will drift without magnetometer

    float q0;
    float q1;
    float q2;
    float q3;

    float rateX;  // deg/s
    float rateY;  // deg/s
    float rateZ;  // deg/s

    float accX;   // g
    float accY;   // g
    float accZ;   // g
};
///////////////// struct za bias/////
struct GyroBias
{

  float gyroBiasX;
  float gyroBiasY;
  float gyroBiasZ;

};
void resetEKF();
void kalmanAngle(float event_accelX, float event_accelY,float event_accelZ,float gyroX,float gyroY,float gyroZ,float dt,kalmanICMData *kalmanICMdata, GyroBias &BiasGyro);
void calibrateGyroBias(int samples,  int16_t readGyroX, int16_t readGyroY,int16_t readGyroZ, GyroBias *BiasOut);
#endif