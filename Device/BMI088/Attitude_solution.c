#include "Attitude_solution.h"
#include "driver_sync.h"
#include <math.h>
#include <string.h>
Attitude_Data_t g_Attitude;
static uint32_t last_sequence, calibration_generation;
static uint8_t have_sequence, reset_yaw;
static float lowpass(float value, float previous, float alpha)
{
    return alpha * previous + (1.0f - alpha) * value;
}
void Attitude_Init(void)
{
    uint32_t key = Driver_Lock();
    memset(&g_Attitude, 0, sizeof(g_Attitude));
    last_sequence = calibration_generation = 0;
    have_sequence = reset_yaw = 0;
    Driver_Unlock(key);
}
void Attitude_Calibrate_ZeroBias(void) { (void)BMI088_RequestCalibration(); }
void Attitude_ResetYaw(void)
{
    uint32_t key = Driver_Lock(); reset_yaw = 1; Driver_Unlock(key);
}
uint8_t Attitude_GetSnapshot(Attitude_Data_t *out)
{
    if (!out) return 0;
    uint32_t key = Driver_Lock(); *out = g_Attitude; Driver_Unlock(key);
    BMI088_Diagnostics d; BMI088_Calibration c;
    BMI088_GetDiagnostics(&d); BMI088_GetCalibration(&c);
    out->Init_OK = d.initialized;
    out->Calibrated = c.state == BMI088_CAL_SUCCEEDED;
    if (!out->Init_OK || !out->Calibrated || d.last_read_status != BMI088_STATUS_OK ||
        (uint32_t)(HAL_GetTick() - out->Last_Valid_Update_ms) >= ATTITUDE_DATA_TIMEOUT_MS)
        out->Data_Valid = 0;
    return 1;
}
void Attitude_ProcessSample(const BMI088_Sample *s, float dt)
{
    Attitude_Data_t a;
    uint32_t key = Driver_Lock(); a = g_Attitude;
    uint8_t yaw_requested = reset_yaw; reset_yaw = 0;
    Driver_Unlock(key);
    if (yaw_requested) a.Angle_Yaw = 0;
    BMI088_Diagnostics d; BMI088_Calibration c;
    BMI088_GetDiagnostics(&d); BMI088_GetCalibration(&c);
    a.Init_OK = d.initialized; a.Calibrated = c.state == BMI088_CAL_SUCCEEDED;
    uint32_t now = HAL_GetTick();
    uint8_t fresh = s && s->communication_status == BMI088_STATUS_OK &&
                    (uint32_t)(now - s->timestamp_ms) < ATTITUDE_DATA_TIMEOUT_MS;
    if (!a.Init_OK || !a.Calibrated || !fresh || d.last_read_status != BMI088_STATUS_OK) {
        a.Data_Valid = 0;
        if (a.Consecutive_Read_Errors < UINT16_MAX) ++a.Consecutive_Read_Errors;
        goto publish;
    }
    if (!isfinite(s->temperature) || !isfinite(s->accel[0]) || !isfinite(s->accel[1]) ||
        !isfinite(s->accel[2]) || !isfinite(s->gyro[0]) || !isfinite(s->gyro[1]) ||
        !isfinite(s->gyro[2]) || s->accel[2] <= 0) {
        a.Data_Valid = 0; goto publish;
    }
    if (have_sequence && s->sequence == last_sequence && c.generation == calibration_generation)
        goto publish; /* Never integrate or refresh the timestamp twice. */
    uint8_t restart = !have_sequence || !a.Data_Valid || c.generation != calibration_generation;
    if (!restart) dt = (uint32_t)(s->timestamp_ms - a.Last_Valid_Update_ms) * 0.001f;
    if (!isfinite(dt) || dt <= 0 || dt > 0.05f) { a.Data_Valid = 0; goto publish; }
    last_sequence = s->sequence; have_sequence = 1;
    calibration_generation = c.generation;
    a.Offset_Accel_X = c.accel_offset[0]; a.Offset_Accel_Y = c.accel_offset[1]; a.Offset_Accel_Z = c.accel_offset[2];
    a.Offset_Gyro_X = c.gyro_offset[0]; a.Offset_Gyro_Y = c.gyro_offset[1]; a.Offset_Gyro_Z = c.gyro_offset[2];
    a.Accel_X = s->accel[0]; a.Accel_Y = s->accel[1]; a.Accel_Z = s->accel[2];
    a.Gyro_X = s->gyro[0]; a.Gyro_Y = s->gyro[1]; a.Gyro_Z = s->gyro[2]; a.Temperature = s->temperature;
    float ax = a.Accel_X - a.Offset_Accel_X, ay = a.Accel_Y - a.Offset_Accel_Y, az = a.Accel_Z - a.Offset_Accel_Z;
    float gx = a.Gyro_X - a.Offset_Gyro_X, gy = a.Gyro_Y - a.Offset_Gyro_Y, gz = a.Gyro_Z - a.Offset_Gyro_Z;
    if (az <= 0) { a.Data_Valid = 0; goto publish; }
    a.flt_ax = restart ? ax : lowpass(ax, a.flt_ax, ACCEL_LPF_ALPHA);
    a.flt_ay = restart ? ay : lowpass(ay, a.flt_ay, ACCEL_LPF_ALPHA);
    a.flt_az = restart ? az : lowpass(az, a.flt_az, ACCEL_LPF_ALPHA);
    a.flt_gx = restart ? gx : lowpass(gx, a.flt_gx, GYRO_LPF_ALPHA);
    a.flt_gy = restart ? gy : lowpass(gy, a.flt_gy, GYRO_LPF_ALPHA);
    a.flt_gz = restart ? gz : lowpass(gz, a.flt_gz, GYRO_LPF_ALPHA);
    float pitch = atan2f(a.flt_ay, sqrtf(a.flt_ax*a.flt_ax + a.flt_az*a.flt_az)) * RAD_TO_DEG;
    float roll = atan2f(-a.flt_ax, sqrtf(a.flt_ay*a.flt_ay + a.flt_az*a.flt_az)) * RAD_TO_DEG;
    if (restart) { a.Angle_Pitch = pitch; a.Angle_Roll = roll; }
    else {
        a.Angle_Pitch += a.flt_gx * RAD_TO_DEG * dt;
        a.Angle_Roll += a.flt_gy * RAD_TO_DEG * dt;
        float norm = sqrtf(a.flt_ax*a.flt_ax + a.flt_ay*a.flt_ay + a.flt_az*a.flt_az);
        if (norm >= ACCEL_NORM_MIN_G*GRAVITY && norm <= ACCEL_NORM_MAX_G*GRAVITY) {
            float alpha = expf(-dt / ACCEL_CORRECTION_TAU_S);
            a.Angle_Pitch = alpha*a.Angle_Pitch + (1-alpha)*pitch;
            a.Angle_Roll = alpha*a.Angle_Roll + (1-alpha)*roll;
        }
        if (fabsf(a.flt_gz) >= YAW_DEADZONE_THRESHOLD) a.Angle_Yaw += a.flt_gz * RAD_TO_DEG * dt;
    }
    a.Data_Valid = isfinite(a.Angle_Pitch) && isfinite(a.Angle_Roll) && isfinite(a.Angle_Yaw);
    if (a.Data_Valid) { a.Last_Valid_Update_ms = s->timestamp_ms; a.Consecutive_Read_Errors = 0; }
publish:
    key = Driver_Lock(); g_Attitude = a; Driver_Unlock(key);
}
void Attitude_Update(float dt)
{
    BMI088_Sample s;
    Attitude_ProcessSample(BMI088_GetSample(&s) ? &s : NULL, dt);
}
uint8_t BMI088Read(void)
{
    /* Initialization lives in main; claiming the owner does no SPI traffic. */
    if (!BMI088_BindTask()) return 0;
    BMI088_Sample sample;
    BMI088_Status status = BMI088_ReadSample(&sample);
    BMI088_CalibrationStep(status == BMI088_STATUS_OK ? &sample : NULL);
    Attitude_ProcessSample(status == BMI088_STATUS_OK ? &sample : NULL,
                           BMI088_SAMPLE_PERIOD_MS * 0.001f);
    Attitude_Data_t attitude;
    return Attitude_GetSnapshot(&attitude) && attitude.Data_Valid;
}
