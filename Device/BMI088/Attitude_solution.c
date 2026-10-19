#include "Attitude_solution.h"
#include "driver_sync.h"
#include <math.h>
#include <string.h>

#define EKF_GYRO_NOISE_VARIANCE       0.0004f
#define EKF_ACCEL_NOISE_VARIANCE      0.01f
#define EKF_INITIAL_COVARIANCE        0.04f
#define EKF_MIN_COVARIANCE            1.0e-9f
#define EKF_MAX_COVARIANCE            4.0f
#define EKF_MAX_INNOVATION            25.0f
#define EKF_MAX_ERROR_ANGLE_RAD       0.50f
#define EKF_ACCEL_MIN_NORM             (ACCEL_NORM_MIN_G * GRAVITY)
#define EKF_ACCEL_MAX_NORM             (ACCEL_NORM_MAX_G * GRAVITY)

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Attitude_Quaternion_t;

Attitude_Data_t g_Attitude;

static uint32_t last_sequence;
static uint32_t calibration_generation;
static uint8_t have_sequence;
static uint8_t reset_yaw;
static uint8_t ekf_initialized;
static Attitude_Quaternion_t ekf_quaternion;
static float ekf_covariance[3][3];

static float lowpass(float value, float previous, float alpha)
{
    return alpha * previous + (1.0f - alpha) * value;
}

static float clamp_float(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static uint8_t quaternion_is_finite(const Attitude_Quaternion_t *q)
{
    return q && isfinite(q->w) && isfinite(q->x) &&
           isfinite(q->y) && isfinite(q->z);
}

static uint8_t quaternion_normalize(Attitude_Quaternion_t *q)
{
    float norm_squared;
    float inverse_norm;

    if (!quaternion_is_finite(q)) return 0;
    norm_squared = q->w * q->w + q->x * q->x +
                   q->y * q->y + q->z * q->z;
    if (!isfinite(norm_squared) || norm_squared < 1.0e-12f) return 0;
    inverse_norm = 1.0f / sqrtf(norm_squared);
    q->w *= inverse_norm;
    q->x *= inverse_norm;
    q->y *= inverse_norm;
    q->z *= inverse_norm;
    return quaternion_is_finite(q);
}

static Attitude_Quaternion_t quaternion_multiply(Attitude_Quaternion_t left,
                                                  Attitude_Quaternion_t right)
{
    Attitude_Quaternion_t result;
    result.w = left.w * right.w - left.x * right.x -
               left.y * right.y - left.z * right.z;
    result.x = left.w * right.x + left.x * right.w +
               left.y * right.z - left.z * right.y;
    result.y = left.w * right.y - left.x * right.z +
               left.y * right.w + left.z * right.x;
    result.z = left.w * right.z + left.x * right.y -
               left.y * right.x + left.z * right.w;
    return result;
}

/* Body-to-world quaternion whose body-frame gravity vector is accel. */
static uint8_t quaternion_from_accel(float ax, float ay, float az,
                                     Attitude_Quaternion_t *out)
{
    float norm;
    float half_w;

    if (!out || !isfinite(ax) || !isfinite(ay) || !isfinite(az)) return 0;
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (!isfinite(norm) || norm <= 1.0e-6f) return 0;
    ax /= norm;
    ay /= norm;
    az /= norm;
    if (az <= -0.9999f) return 0;

    half_w = sqrtf((1.0f + az) * 0.5f);
    if (!isfinite(half_w) || half_w <= 1.0e-6f) return 0;
    out->w = half_w;
    out->x = ay / (2.0f * half_w);
    out->y = -ax / (2.0f * half_w);
    out->z = 0.0f;
    return quaternion_normalize(out);
}

static Attitude_Quaternion_t quaternion_from_euler_xyz(float angle_x,
                                                        float angle_y,
                                                        float angle_z)
{
    Attitude_Quaternion_t result;
    float cx = cosf(0.5f * angle_x);
    float sx = sinf(0.5f * angle_x);
    float cy = cosf(0.5f * angle_y);
    float sy = sinf(0.5f * angle_y);
    float cz = cosf(0.5f * angle_z);
    float sz = sinf(0.5f * angle_z);

    result.w = cx * cy * cz + sx * sy * sz;
    result.x = sx * cy * cz - cx * sy * sz;
    result.y = cx * sy * cz + sx * cy * sz;
    result.z = cx * cy * sz - sx * sy * cz;
    (void)quaternion_normalize(&result);
    return result;
}

static uint8_t quaternion_to_euler(const Attitude_Quaternion_t *q,
                                   float *angle_x, float *angle_y,
                                   float *angle_z)
{
    float sin_y;

    if (!quaternion_is_finite(q) || !angle_x || !angle_y || !angle_z) return 0;
    *angle_x = atan2f(2.0f * (q->w * q->x + q->y * q->z),
                      1.0f - 2.0f * (q->x * q->x + q->y * q->y));
    sin_y = 2.0f * (q->w * q->y - q->z * q->x);
    sin_y = clamp_float(sin_y, -1.0f, 1.0f);
    *angle_y = asinf(sin_y);
    *angle_z = atan2f(2.0f * (q->w * q->z + q->x * q->y),
                      1.0f - 2.0f * (q->y * q->y + q->z * q->z));
    return isfinite(*angle_x) && isfinite(*angle_y) && isfinite(*angle_z);
}

static uint8_t quaternion_propagate(Attitude_Quaternion_t *q,
                                    float gx, float gy, float gz, float dt)
{
    Attitude_Quaternion_t omega;
    Attitude_Quaternion_t derivative;

    if (!q || !isfinite(gx) || !isfinite(gy) || !isfinite(gz) ||
        !isfinite(dt) || dt <= 0.0f) return 0;
    omega.w = 0.0f;
    omega.x = gx;
    omega.y = gy;
    omega.z = gz;
    derivative = quaternion_multiply(*q, omega);
    q->w += 0.5f * derivative.w * dt;
    q->x += 0.5f * derivative.x * dt;
    q->y += 0.5f * derivative.y * dt;
    q->z += 0.5f * derivative.z * dt;
    return quaternion_normalize(q);
}

static void ekf_reset_covariance(void)
{
    memset(ekf_covariance, 0, sizeof(ekf_covariance));
    ekf_covariance[0][0] = EKF_INITIAL_COVARIANCE;
    ekf_covariance[1][1] = EKF_INITIAL_COVARIANCE;
    ekf_covariance[2][2] = EKF_INITIAL_COVARIANCE;
}

static uint8_t covariance_is_finite(void)
{
    unsigned row;
    unsigned column;
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            if (!isfinite(ekf_covariance[row][column])) return 0;
        }
    }
    return 1;
}

static void covariance_symmetrize_and_bound(void)
{
    unsigned row;
    unsigned column;
    float value;

    for (row = 0; row < 3; ++row) {
        for (column = row + 1; column < 3; ++column) {
            value = 0.5f * (ekf_covariance[row][column] +
                            ekf_covariance[column][row]);
            ekf_covariance[row][column] = value;
            ekf_covariance[column][row] = value;
        }
        if (!isfinite(ekf_covariance[row][row])) {
            ekf_covariance[row][row] = EKF_INITIAL_COVARIANCE;
        }
        if (ekf_covariance[row][row] < EKF_MIN_COVARIANCE) {
            ekf_covariance[row][row] = EKF_MIN_COVARIANCE;
        } else if (ekf_covariance[row][row] > EKF_MAX_COVARIANCE) {
            ekf_covariance[row][row] = EKF_MAX_COVARIANCE;
        }
    }
}

static void ekf_predict_covariance(float gx, float gy, float gz, float dt)
{
    float f[3][3];
    float temporary[3][3] = {{0.0f}};
    float predicted[3][3] = {{0.0f}};
    float process_variance = EKF_GYRO_NOISE_VARIANCE * dt * dt;
    unsigned row;
    unsigned column;
    unsigned inner;

    f[0][0] = 1.0f;     f[0][1] = gz * dt;  f[0][2] = -gy * dt;
    f[1][0] = -gz * dt; f[1][1] = 1.0f;     f[1][2] = gx * dt;
    f[2][0] = gy * dt;  f[2][1] = -gx * dt; f[2][2] = 1.0f;

    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            for (inner = 0; inner < 3; ++inner) {
                temporary[row][column] += f[row][inner] *
                                          ekf_covariance[inner][column];
            }
        }
    }
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            for (inner = 0; inner < 3; ++inner) {
                predicted[row][column] += temporary[row][inner] * f[column][inner];
            }
            if (row == column) predicted[row][column] += process_variance;
        }
    }
    memcpy(ekf_covariance, predicted, sizeof(ekf_covariance));
    covariance_symmetrize_and_bound();
}

static uint8_t inverse_3x3(const float matrix[3][3], float inverse[3][3])
{
    float determinant;

    determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] -
                                  matrix[1][2] * matrix[2][1])
                - matrix[0][1] * (matrix[1][0] * matrix[2][2] -
                                  matrix[1][2] * matrix[2][0])
                + matrix[0][2] * (matrix[1][0] * matrix[2][1] -
                                  matrix[1][1] * matrix[2][0]);
    if (!isfinite(determinant) || fabsf(determinant) < 1.0e-9f) return 0;

    inverse[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) / determinant;
    inverse[0][1] = (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) / determinant;
    inverse[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) / determinant;
    inverse[1][0] = (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) / determinant;
    inverse[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) / determinant;
    inverse[1][2] = (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) / determinant;
    inverse[2][0] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) / determinant;
    inverse[2][1] = (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) / determinant;
    inverse[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) / determinant;
    return 1;
}

static uint8_t ekf_update_accel(Attitude_Quaternion_t *q,
                                float ax, float ay, float az,
                                Attitude_Data_t *attitude)
{
    float gravity[3];
    float measurement[3];
    float innovation[3];
    float h[3][3];
    float innovation_covariance[3][3];
    float inverse_innovation_covariance[3][3];
    float kalman_gain[3][3] = {{0.0f}};
    float identity_minus_kh[3][3];
    float covariance_before[3][3];
    float covariance_after[3][3] = {{0.0f}};
    float temporary[3][3] = {{0.0f}};
    float correction[3] = {0.0f};
    float mahalanobis;
    float correction_norm;
    unsigned row;
    unsigned column;
    unsigned inner;

    if (!q || !attitude || !quaternion_is_finite(q)) return 0;
    measurement[0] = ax;
    measurement[1] = ay;
    measurement[2] = az;
    gravity[0] = 2.0f * (q->x * q->z - q->w * q->y);
    gravity[1] = 2.0f * (q->w * q->x + q->y * q->z);
    gravity[2] = q->w * q->w - q->x * q->x -
                 q->y * q->y + q->z * q->z;

    /* H = [gravity]x for a right-multiplicative body-frame error angle. */
    h[0][0] = 0.0f;        h[0][1] = -gravity[2]; h[0][2] = gravity[1];
    h[1][0] = gravity[2];  h[1][1] = 0.0f;        h[1][2] = -gravity[0];
    h[2][0] = -gravity[1]; h[2][1] = gravity[0];  h[2][2] = 0.0f;
    for (row = 0; row < 3; ++row) {
        innovation[row] = measurement[row] - gravity[row];
        if (row == 0) attitude->EKF_Innovation_X = innovation[row];
        if (row == 1) attitude->EKF_Innovation_Y = innovation[row];
        if (row == 2) attitude->EKF_Innovation_Z = innovation[row];
    }

    memcpy(covariance_before, ekf_covariance, sizeof(covariance_before));
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            innovation_covariance[row][column] = EKF_ACCEL_NOISE_VARIANCE;
            for (inner = 0; inner < 3; ++inner) {
                unsigned covariance_column;
                for (covariance_column = 0; covariance_column < 3;
                     ++covariance_column) {
                    innovation_covariance[row][column] +=
                        h[row][inner] * covariance_before[inner][covariance_column] *
                        h[column][covariance_column];
                }
            }
        }
    }
    if (!inverse_3x3(innovation_covariance, inverse_innovation_covariance)) return 0;

    mahalanobis = 0.0f;
    for (row = 0; row < 3; ++row) {
        float weighted_innovation = 0.0f;
        for (column = 0; column < 3; ++column) {
            weighted_innovation += inverse_innovation_covariance[row][column] *
                                   innovation[column];
        }
        mahalanobis += innovation[row] * weighted_innovation;
    }
    if (!isfinite(mahalanobis) || mahalanobis > EKF_MAX_INNOVATION) return 0;

    /* K = P H^T S^-1. */
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            for (inner = 0; inner < 3; ++inner) {
                unsigned measurement_axis;
                for (measurement_axis = 0; measurement_axis < 3; ++measurement_axis) {
                    kalman_gain[row][column] += covariance_before[row][inner] *
                        h[measurement_axis][inner] *
                        inverse_innovation_covariance[measurement_axis][column];
                }
            }
        }
    }
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            correction[row] += kalman_gain[row][column] * innovation[column];
        }
    }
    correction_norm = sqrtf(correction[0] * correction[0] +
                            correction[1] * correction[1] +
                            correction[2] * correction[2]);
    if (!isfinite(correction_norm)) return 0;
    if (correction_norm > EKF_MAX_ERROR_ANGLE_RAD) {
        float scale = EKF_MAX_ERROR_ANGLE_RAD / correction_norm;
        correction[0] *= scale;
        correction[1] *= scale;
        correction[2] *= scale;
    }

    {
        Attitude_Quaternion_t error_quaternion;
        Attitude_Quaternion_t corrected;
        error_quaternion.w = 1.0f;
        error_quaternion.x = 0.5f * correction[0];
        error_quaternion.y = 0.5f * correction[1];
        error_quaternion.z = 0.5f * correction[2];
        if (!quaternion_normalize(&error_quaternion)) return 0;
        corrected = quaternion_multiply(*q, error_quaternion);
        if (!quaternion_normalize(&corrected)) return 0;
        *q = corrected;
    }

    /* Joseph form: P = (I-KH) P (I-KH)^T + K R K^T. */
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            identity_minus_kh[row][column] = (row == column) ? 1.0f : 0.0f;
            for (inner = 0; inner < 3; ++inner) {
                identity_minus_kh[row][column] -= kalman_gain[row][inner] *
                                                   h[inner][column];
            }
        }
    }
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            for (inner = 0; inner < 3; ++inner) {
                temporary[row][column] += identity_minus_kh[row][inner] *
                                           covariance_before[inner][column];
            }
        }
    }
    for (row = 0; row < 3; ++row) {
        for (column = 0; column < 3; ++column) {
            for (inner = 0; inner < 3; ++inner) {
                covariance_after[row][column] += temporary[row][inner] *
                                                 identity_minus_kh[column][inner];
                covariance_after[row][column] += kalman_gain[row][inner] *
                    EKF_ACCEL_NOISE_VARIANCE * kalman_gain[column][inner];
            }
        }
    }
    memcpy(ekf_covariance, covariance_after, sizeof(ekf_covariance));
    covariance_symmetrize_and_bound();
    return covariance_is_finite() && quaternion_is_finite(q);
}

static void publish_filter_state(Attitude_Data_t *attitude)
{
    float angle_x;
    float angle_y;
    float angle_z;
    unsigned i;

    attitude->Quaternion_W = ekf_quaternion.w;
    attitude->Quaternion_X = ekf_quaternion.x;
    attitude->Quaternion_Y = ekf_quaternion.y;
    attitude->Quaternion_Z = ekf_quaternion.z;
    attitude->EKF_Covariance_X = ekf_covariance[0][0];
    attitude->EKF_Covariance_Y = ekf_covariance[1][1];
    attitude->EKF_Covariance_Z = ekf_covariance[2][2];
    attitude->EKF_Valid = ekf_initialized && quaternion_is_finite(&ekf_quaternion) &&
                          covariance_is_finite();
    if (!attitude->EKF_Valid || !quaternion_to_euler(&ekf_quaternion,
                                                     &angle_x, &angle_y, &angle_z)) {
        attitude->EKF_Valid = 0;
        return;
    }
    attitude->Angle_Pitch = angle_x * RAD_TO_DEG;
    attitude->Angle_Roll = angle_y * RAD_TO_DEG;
    attitude->Angle_Yaw = angle_z * RAD_TO_DEG;
    for (i = 0; i < 3; ++i) {
        if (!isfinite(ekf_covariance[i][i])) attitude->EKF_Valid = 0;
    }
}

void Attitude_Init(void)
{
    uint32_t key = Driver_Lock();
    memset(&g_Attitude, 0, sizeof(g_Attitude));
    last_sequence = calibration_generation = 0;
    have_sequence = reset_yaw = 0;
    ekf_initialized = 0;
    ekf_quaternion.w = 1.0f;
    ekf_quaternion.x = 0.0f;
    ekf_quaternion.y = 0.0f;
    ekf_quaternion.z = 0.0f;
    ekf_reset_covariance();
    Driver_Unlock(key);
}

void Attitude_Calibrate_ZeroBias(void)
{
    (void)BMI088_RequestCalibration();
}

void Attitude_ResetYaw(void)
{
    uint32_t key = Driver_Lock();
    reset_yaw = 1;
    Driver_Unlock(key);
}

uint8_t Attitude_GetSnapshot(Attitude_Data_t *out)
{
    if (!out) return 0;
    {
        uint32_t key = Driver_Lock();
        *out = g_Attitude;
        Driver_Unlock(key);
    }
    {
        BMI088_Diagnostics diagnostics;
        BMI088_Calibration calibration;
        BMI088_GetDiagnostics(&diagnostics);
        BMI088_GetCalibration(&calibration);
        out->Init_OK = diagnostics.initialized;
        out->Calibrated = calibration.state == BMI088_CAL_SUCCEEDED;
        if (!out->Init_OK || !out->Calibrated ||
            diagnostics.last_read_status != BMI088_STATUS_OK ||
            !out->EKF_Valid ||
            (uint32_t)(HAL_GetTick() - out->Last_Valid_Update_ms) >=
                ATTITUDE_DATA_TIMEOUT_MS) {
            out->Data_Valid = 0;
        }
    }
    return 1;
}

void Attitude_ProcessSample(const BMI088_Sample *s, float dt)
{
    Attitude_Data_t attitude;
    BMI088_Diagnostics diagnostics;
    BMI088_Calibration calibration;
    uint32_t key;
    uint8_t yaw_requested;
    uint8_t restart;
    uint8_t acceleration_valid;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float acceleration_norm;
    uint32_t now;

    key = Driver_Lock();
    attitude = g_Attitude;
    yaw_requested = reset_yaw;
    reset_yaw = 0;
    Driver_Unlock(key);
    BMI088_GetDiagnostics(&diagnostics);
    BMI088_GetCalibration(&calibration);
    attitude.Init_OK = diagnostics.initialized;
    attitude.Calibrated = calibration.state == BMI088_CAL_SUCCEEDED;
    now = HAL_GetTick();

    if (!attitude.Init_OK || !attitude.Calibrated || !s ||
        s->communication_status != BMI088_STATUS_OK ||
        (uint32_t)(now - s->timestamp_ms) >= ATTITUDE_DATA_TIMEOUT_MS ||
        diagnostics.last_read_status != BMI088_STATUS_OK) {
        attitude.Data_Valid = 0;
        attitude.EKF_Valid = 0;
        if (attitude.Consecutive_Read_Errors < UINT16_MAX) {
            ++attitude.Consecutive_Read_Errors;
        }
        goto publish;
    }
    if (!isfinite(s->temperature) || !isfinite(s->accel[0]) ||
        !isfinite(s->accel[1]) || !isfinite(s->accel[2]) ||
        !isfinite(s->gyro[0]) || !isfinite(s->gyro[1]) ||
        !isfinite(s->gyro[2]) || s->accel[2] <= 0.0f) {
        attitude.Data_Valid = 0;
        attitude.EKF_Valid = 0;
        goto publish;
    }
    if (have_sequence && s->sequence == last_sequence &&
        calibration.generation == calibration_generation) {
        goto publish; /* Do not integrate or refresh the timestamp twice. */
    }

    restart = !have_sequence || !attitude.Data_Valid ||
              calibration.generation != calibration_generation;
    if (!restart) {
        dt = (uint32_t)(s->timestamp_ms - attitude.Last_Valid_Update_ms) * 0.001f;
    }
    if (!isfinite(dt) || dt <= 0.0f || dt > 0.05f) {
        attitude.Data_Valid = 0;
        attitude.EKF_Valid = 0;
        goto publish;
    }

    last_sequence = s->sequence;
    have_sequence = 1;
    calibration_generation = calibration.generation;
    attitude.Offset_Accel_X = calibration.accel_offset[0];
    attitude.Offset_Accel_Y = calibration.accel_offset[1];
    attitude.Offset_Accel_Z = calibration.accel_offset[2];
    attitude.Offset_Gyro_X = calibration.gyro_offset[0];
    attitude.Offset_Gyro_Y = calibration.gyro_offset[1];
    attitude.Offset_Gyro_Z = calibration.gyro_offset[2];
    attitude.Accel_X = s->accel[0];
    attitude.Accel_Y = s->accel[1];
    attitude.Accel_Z = s->accel[2];
    attitude.Gyro_X = s->gyro[0];
    attitude.Gyro_Y = s->gyro[1];
    attitude.Gyro_Z = s->gyro[2];
    attitude.Temperature = s->temperature;

    ax = attitude.Accel_X - attitude.Offset_Accel_X;
    ay = attitude.Accel_Y - attitude.Offset_Accel_Y;
    az = attitude.Accel_Z - attitude.Offset_Accel_Z;
    gx = attitude.Gyro_X - attitude.Offset_Gyro_X;
    gy = attitude.Gyro_Y - attitude.Offset_Gyro_Y;
    gz = attitude.Gyro_Z - attitude.Offset_Gyro_Z;
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az) ||
        !isfinite(gx) || !isfinite(gy) || !isfinite(gz) || az <= 0.0f) {
        attitude.Data_Valid = 0;
        attitude.EKF_Valid = 0;
        goto publish;
    }

    attitude.flt_ax = restart ? ax : lowpass(ax, attitude.flt_ax, ACCEL_LPF_ALPHA);
    attitude.flt_ay = restart ? ay : lowpass(ay, attitude.flt_ay, ACCEL_LPF_ALPHA);
    attitude.flt_az = restart ? az : lowpass(az, attitude.flt_az, ACCEL_LPF_ALPHA);
    attitude.flt_gx = restart ? gx : lowpass(gx, attitude.flt_gx, GYRO_LPF_ALPHA);
    attitude.flt_gy = restart ? gy : lowpass(gy, attitude.flt_gy, GYRO_LPF_ALPHA);
    attitude.flt_gz = restart ? gz : lowpass(gz, attitude.flt_gz, GYRO_LPF_ALPHA);
    ax = attitude.flt_ax;
    ay = attitude.flt_ay;
    az = attitude.flt_az;
    gx = attitude.flt_gx;
    gy = attitude.flt_gy;
    gz = attitude.flt_gz;
    acceleration_norm = sqrtf(ax * ax + ay * ay + az * az);
    acceleration_valid = isfinite(acceleration_norm) &&
                         acceleration_norm >= EKF_ACCEL_MIN_NORM &&
                         acceleration_norm <= EKF_ACCEL_MAX_NORM;
    attitude.EKF_Innovation_X = 0.0f;
    attitude.EKF_Innovation_Y = 0.0f;
    attitude.EKF_Innovation_Z = 0.0f;

    if (restart) {
        if (!acceleration_valid || !quaternion_from_accel(ax, ay, az,
                                                          &ekf_quaternion)) {
            attitude.Data_Valid = 0;
            attitude.EKF_Valid = 0;
            goto publish;
        }
        ekf_reset_covariance();
        ekf_initialized = 1;
    } else {
        if (!ekf_initialized || !quaternion_propagate(&ekf_quaternion,
                                                       gx, gy, gz, dt)) {
            attitude.Data_Valid = 0;
            attitude.EKF_Valid = 0;
            goto publish;
        }
        ekf_predict_covariance(gx, gy, gz, dt);
    }

    if (yaw_requested && ekf_initialized) {
        float angle_x;
        float angle_y;
        float angle_z;
        if (quaternion_to_euler(&ekf_quaternion, &angle_x, &angle_y, &angle_z)) {
            ekf_quaternion = quaternion_from_euler_xyz(angle_x, angle_y, 0.0f);
            ekf_covariance[0][2] = 0.0f;
            ekf_covariance[2][0] = 0.0f;
            ekf_covariance[1][2] = 0.0f;
            ekf_covariance[2][1] = 0.0f;
            ekf_covariance[2][2] = EKF_INITIAL_COVARIANCE;
        }
    }

    if (acceleration_valid) {
        ax /= acceleration_norm;
        ay /= acceleration_norm;
        az /= acceleration_norm;
        (void)ekf_update_accel(&ekf_quaternion, ax, ay, az, &attitude);
    }
    publish_filter_state(&attitude);
    attitude.Data_Valid = attitude.EKF_Valid &&
                          isfinite(attitude.Angle_Pitch) &&
                          isfinite(attitude.Angle_Roll) &&
                          isfinite(attitude.Angle_Yaw);
    if (attitude.Data_Valid) {
        attitude.Last_Valid_Update_ms = s->timestamp_ms;
        attitude.Consecutive_Read_Errors = 0;
    }
publish:
    key = Driver_Lock();
    g_Attitude = attitude;
    Driver_Unlock(key);
}

void Attitude_Update(float dt)
{
    BMI088_Sample sample;
    Attitude_ProcessSample(BMI088_GetSample(&sample) ? &sample : NULL, dt);
}

uint8_t BMI088Read(void)
{
    BMI088_Sample sample;
    Attitude_Data_t attitude;

    /* Initialization lives in main; claiming the owner does no SPI traffic. */
    if (!BMI088_BindTask()) return 0;
    if (BMI088_ReadSample(&sample) != BMI088_STATUS_OK) {
        BMI088_CalibrationStep(NULL);
        Attitude_ProcessSample(NULL, BMI088_SAMPLE_PERIOD_MS * 0.001f);
        return 0;
    }
    BMI088_CalibrationStep(&sample);
    Attitude_ProcessSample(&sample, BMI088_SAMPLE_PERIOD_MS * 0.001f);
    return Attitude_GetSnapshot(&attitude) && attitude.Data_Valid;
}
