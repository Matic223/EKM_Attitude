#include "EKM_att.h"
#include <math.h>

// Quaternion state: x = [q0 q1 q2 q3]
static float x[4] = {1.0f, 0.0f, 0.0f, 0.0f};

// Covariance matrix P, 4x4
static float P[4][4] = {
    {0.05f, 0.0f, 0.0f, 0.0f},
    {0.0f, 0.05f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.05f, 0.0f},
    {0.0f, 0.0f, 0.0f, 0.05f}
};

// Process noise Q, 4x4
static float Q_tunnable= 1e-5f;
static float Q[4][4] = {
    {Q_tunnable, 0.0f, 0.0f, 0.0f},
    {0.0f, Q_tunnable, 0.0f, 0.0f},
    {0.0f, 0.0f, Q_tunnable, 0.0f},
    {0.0f, 0.0f, 0.0f, Q_tunnable}
};

// Accelerometer measurement noise R, 3x3
static float R_tunnable= 0.20f;
static float R[3][3] = {
    {R_tunnable, 0.0f, 0.0f},
    {0.0f, R_tunnable, 0.0f},
    {0.0f, 0.0f, R_tunnable}
};

// Calibration variables
static float sumX = 0.0f;
static float sumY = 0.0f;
static float sumZ = 0.0f;
static int count = 0;

// Latest converted values
static float AccX, AccY, AccZ; // g
static float gx, gy, gz;       // deg/s

static void normalizeQuaternion()
{
    float norm = sqrtf(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);

    if (norm > 1e-6f) {
        x[0] /= norm;
        x[1] /= norm;
        x[2] /= norm;
        x[3] /= norm;
    } else {
        x[0] = 1.0f;
        x[1] = 0.0f;
        x[2] = 0.0f;
        x[3] = 0.0f;
    }
}

void resetEKF()
{
    x[0] = 1.0f;
    x[1] = 0.0f;
    x[2] = 0.0f;
    x[3] = 0.0f;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = 0.0f;
        }
        P[i][i] = 0.05f;
    }
}

void calibrateGyroBias(
    int samples,
    int16_t readGyroX,
    int16_t readGyroY,
    int16_t readGyroZ,
    GyroBias *BiasOut
) {
    float gxCal = (float)readGyroX / 16.4f;
    float gyCal = (float)readGyroY / 16.4f;
    float gzCal = (float)readGyroZ / 16.4f;

    sumX += gxCal;
    sumY += gyCal;
    sumZ += gzCal;

    count++;

    if (count >= samples) {
        BiasOut->gyroBiasX = sumX / samples;
        BiasOut->gyroBiasY = sumY / samples;
        BiasOut->gyroBiasZ = sumZ / samples;

        sumX = 0.0f;
        sumY = 0.0f;
        sumZ = 0.0f;
        count = 0;
    }
}

static void buildPicardPhi(float wx, float wy, float wz, float dt, float Phi[4][4])
{
    // wx, wy, wz in rad/s
    float dx = wx * dt;
    float dy = wy * dt;
    float dz = wz * dt;

    float d2 = dx*dx + dy*dy + dz*dz;
    float d4 = d2*d2;

    float a = 1.0f - d2 / 8.0f + d4 / 384.0f;
    float b = 0.5f - d2 / 48.0f;

    Phi[0][0] = a;      Phi[0][1] = -b*dx;  Phi[0][2] = -b*dy;  Phi[0][3] = -b*dz;
    Phi[1][0] = b*dx;   Phi[1][1] = a;      Phi[1][2] = b*dz;   Phi[1][3] = -b*dy;
    Phi[2][0] = b*dy;   Phi[2][1] = -b*dz;  Phi[2][2] = a;      Phi[2][3] = b*dx;
    Phi[3][0] = b*dz;   Phi[3][1] = b*dy;   Phi[3][2] = -b*dx;  Phi[3][3] = a;
}

static void predictState(const float Phi[4][4])
{
    float xNew[4];

    for (int i = 0; i < 4; i++) {
        xNew[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            xNew[i] += Phi[i][j] * x[j];
        }
    }

    for (int i = 0; i < 4; i++) {
        x[i] = xNew[i];
    }

    normalizeQuaternion();
}

static void predictCovariance(const float F[4][4])
{
    float FP[4][4] = {0};
    float FPFt[4][4] = {0};

    // FP = F * P
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            FP[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    // FPFt = FP * F^T
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            FPFt[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                FPFt[i][j] += FP[i][k] * F[j][k];
            }
        }
    }

    // P = FPFt + Q
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = FPFt[i][j] + Q[i][j];
        }
    }
}

static void predictGravity(float h[3])
{
    float q0 = x[0];
    float q1 = x[1];
    float q2 = x[2];
    float q3 = x[3];

    h[0] = 2.0f * (q1*q3 - q0*q2);
    h[1] = 2.0f * (q0*q1 + q2*q3);
    h[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

static void computeH(float H[3][4])
{
    float q0 = x[0];
    float q1 = x[1];
    float q2 = x[2];
    float q3 = x[3];

    H[0][0] = -2.0f*q2;
    H[0][1] =  2.0f*q3;
    H[0][2] = -2.0f*q0;
    H[0][3] =  2.0f*q1;

    H[1][0] =  2.0f*q1;
    H[1][1] =  2.0f*q0;
    H[1][2] =  2.0f*q3;
    H[1][3] =  2.0f*q2;

    H[2][0] =  2.0f*q0;
    H[2][1] = -2.0f*q1;
    H[2][2] = -2.0f*q2;
    H[2][3] =  2.0f*q3;
}

static bool invert3x3(const float A[3][3], float invA[3][3])
{
    float det =
        A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]) -
        A[0][1]*(A[1][0]*A[2][2] - A[1][2]*A[2][0]) +
        A[0][2]*(A[1][0]*A[2][1] - A[1][1]*A[2][0]);

    if (fabsf(det) < 1e-9f) {
        return false;
    }

    float invDet = 1.0f / det;

    invA[0][0] =  (A[1][1]*A[2][2] - A[1][2]*A[2][1]) * invDet;
    invA[0][1] = -(A[0][1]*A[2][2] - A[0][2]*A[2][1]) * invDet;
    invA[0][2] =  (A[0][1]*A[1][2] - A[0][2]*A[1][1]) * invDet;

    invA[1][0] = -(A[1][0]*A[2][2] - A[1][2]*A[2][0]) * invDet;
    invA[1][1] =  (A[0][0]*A[2][2] - A[0][2]*A[2][0]) * invDet;
    invA[1][2] = -(A[0][0]*A[1][2] - A[0][2]*A[1][0]) * invDet;

    invA[2][0] =  (A[1][0]*A[2][1] - A[1][1]*A[2][0]) * invDet;
    invA[2][1] = -(A[0][0]*A[2][1] - A[0][1]*A[2][0]) * invDet;
    invA[2][2] =  (A[0][0]*A[1][1] - A[0][1]*A[1][0]) * invDet;

    return true;
}

static void ekfAccelerometerUpdate(float ax, float ay, float az)
{
    float normA = sqrtf(ax*ax + ay*ay + az*az);

    // Use accelerometer only when it is close to 1 g
    if (normA < 0.8f || normA > 1.2f) {
        return;
    }

    float z[3] = {
        ax / normA,
        ay / normA,
        az / normA
    };

    float h[3];
    predictGravity(h);

    float y[3] = {
        z[0] - h[0],
        z[1] - h[1],
        z[2] - h[2]
    };

    float H[3][4];
    computeH(H);

    // PHt = P * H^T, size 4x3
    float PHt[4][3] = {0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            PHt[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                PHt[i][j] += P[i][k] * H[j][k];
            }
        }
    }

    // S = H * P * H^T + R, size 3x3
    float S[3][3] = {0};

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            S[i][j] = R[i][j];
            for (int k = 0; k < 4; k++) {
                S[i][j] += H[i][k] * PHt[k][j];
            }
        }
    }

    float Sinv[3][3];

    if (!invert3x3(S, Sinv)) {
        return;
    }

    // K = PHt * S^-1, size 4x3
    float K[4][3] = {0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                K[i][j] += PHt[i][k] * Sinv[k][j];
            }
        }
    }

    // x = x + K*y
    for (int i = 0; i < 4; i++) {
        float correction = 0.0f;
        for (int j = 0; j < 3; j++) {
            correction += K[i][j] * y[j];
        }
        x[i] += correction;
    }

    normalizeQuaternion();

    // P = (I - K*H) * P
    float KH[4][4] = {0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            KH[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }

    float IminusKH[4][4];

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            IminusKH[i][j] = -KH[i][j];
        }
        IminusKH[i][i] += 1.0f;
    }

    float newP[4][4] = {0};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            newP[i][j] = 0.0f;
            for (int k = 0; k < 4; k++) {
                newP[i][j] += IminusKH[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = newP[i][j];
        }
    }
}
float pitch180FromQuaternion(float q0, float q1, float q2, float q3)
{
    // Body X axis expressed in world frame
    float bx_x = 1.0f - 2.0f * (q2*q2 + q3*q3);
    float bx_z = 2.0f * (q1*q3 - q0*q2);

    // Pitch-like angle of body X axis
    return atan2f(-bx_z, bx_x) * RAD_TO_DEG;
}
static void quaternionToEuler(float &roll, float &pitch, float &yaw)
{
    float q0 = x[0];
    float q1 = x[1];
    float q2 = x[2];
    float q3 = x[3];

    roll = atan2f(
        2.0f * (q0*q1 + q2*q3),
        1.0f - 2.0f * (q1*q1 + q2*q2)
    ) * RAD_TO_DEG;

    float sinp = 2.0f * (q0*q2 - q3*q1);

    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    pitch = asinf(sinp) * RAD_TO_DEG;
    
   //pitch = pitch180FromQuaternion(q0, q1, q2, q3);

    yaw = atan2f(
        2.0f * (q0*q3 + q1*q2),
        1.0f - 2.0f * (q2*q2 + q3*q3)
    ) * RAD_TO_DEG;
}

void kalmanAngle(
    float event_accelX,
    float event_accelY,
    float event_accelZ,
    float gyroX,
    float gyroY,
    float gyroZ,
    float dt,
    kalmanICMData *ekfData,
    GyroBias &BiasGyro
) {
    if (ekfData == nullptr) {
        return;
    }

    if (dt <= 0.0f || dt > 0.05f) {
        return;
    }

    // 1. Convert accelerometer raw LSB to g
    AccX =( event_accelX / 2048.0f);
    AccY = event_accelY / 2048.0f;
    AccZ = (event_accelZ / 2048.0f);

    // 2. Convert gyro raw LSB to deg/s and subtract bias
    gx =( gyroX / 16.4f - BiasGyro.gyroBiasX);
    gy = gyroY / 16.4f - BiasGyro.gyroBiasY;
    gz = (gyroZ / 16.4f - BiasGyro.gyroBiasZ);

    // 3. Convert gyro to rad/s for quaternion math
    float wx = gx * DEG_TO_RAD;
    float wy = gy * DEG_TO_RAD;
    float wz = gz * DEG_TO_RAD;

    // 4. Build Picard propagation matrix
    float Phi[4][4];
    buildPicardPhi(wx, wy, wz, dt, Phi);

    // 5. Predict state
    predictState(Phi);

    // 6. Predict covariance, F = Phi
    predictCovariance(Phi);

    // 7. Accelerometer EKF update
    ekfAccelerometerUpdate(AccX, AccY, AccZ);

    // 8. Output Euler angles
    float roll, pitch, yaw;
    quaternionToEuler(roll, pitch, yaw);

    ekfData->roll = roll;
    ekfData->pitch = pitch;
    ekfData->yaw = yaw;

    ekfData->q0 = x[0];
    ekfData->q1 = x[1];
    ekfData->q2 = x[2];
    ekfData->q3 = x[3];

    ekfData->rateX = gx;
    ekfData->rateY = gy;
    ekfData->rateZ = gz;

    ekfData->accX = AccX;
    ekfData->accY = AccY;
    ekfData->accZ = AccZ;
}