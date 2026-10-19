#ifndef __ATTITUDE_SOLUTION_H
#define __ATTITUDE_SOLUTION_H

#include "stdint.h"
#include "BMI088driver.h"

#define RAD_TO_DEG    57.2957795f
#define DEG_TO_RAD    0.017453292f
#define GRAVITY       9.80665f

#define GYRO_LPF_ALPHA            0.50f
#define ACCEL_LPF_ALPHA           0.55f
#define YAW_DEADZONE_THRESHOLD    0.002617994f
#define ACCEL_CORRECTION_TAU_S    0.50f
#define ACCEL_NORM_MIN_G          0.85f
#define ACCEL_NORM_MAX_G          1.15f
#define ATTITUDE_DATA_TIMEOUT_MS  50U

typedef struct {
    uint8_t Init_OK;
    uint8_t Calibrated;
    uint8_t Data_Valid;
    uint16_t Consecutive_Read_Errors;
    uint32_t Last_Valid_Update_ms;

    float Accel_X;
    float Accel_Y;
    float Accel_Z;
    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;
    float Temperature;

    float Offset_Accel_X;
    float Offset_Accel_Y;
    float Offset_Accel_Z;
    float Offset_Gyro_X;
    float Offset_Gyro_Y;
    float Offset_Gyro_Z;

    float Angle_Pitch;
    float Angle_Roll;
    float Angle_Yaw;

    /* Quaternion attitude, ordered as w, x, y, z. */
    float Quaternion_W;
    float Quaternion_X;
    float Quaternion_Y;
    float Quaternion_Z;
    uint8_t EKF_Valid;
    /* Covariance diagonal and innovation are ordered by body X/Y/Z error. */
    float EKF_Covariance_X;
    float EKF_Covariance_Y;
    float EKF_Covariance_Z;
    float EKF_Innovation_X;
    float EKF_Innovation_Y;
    float EKF_Innovation_Z;

    float flt_ax;
    float flt_ay;
    float flt_az;
    float flt_gx;
    float flt_gy;
    float flt_gz;
} Attitude_Data_t;

void Attitude_Init(void);
void Attitude_Calibrate_ZeroBias(void);
void Attitude_Update(float dt);
void Attitude_ResetYaw(void);
uint8_t Attitude_GetSnapshot(Attitude_Data_t *out);
/* BMI088 task only. BMI088Read owns both sampling and the attitude algorithm. */
void Attitude_ProcessSample(const BMI088_Sample *sample, float dt);

extern Attitude_Data_t g_Attitude;

#endif
