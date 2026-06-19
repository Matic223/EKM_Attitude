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

void resetEKF();

void kalmanAngle(
    float AccX,
    float AccY,
    float AccZ,
    float gx_dps,
    float gy_dps,
    float gz_dps,
    float dt,
    kalmanICMData *ekfData
);


#endif