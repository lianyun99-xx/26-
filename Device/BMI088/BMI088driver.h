#ifndef BMI088_DRIVER_H
#define BMI088_DRIVER_H
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#define BMI088_SAMPLE_PERIOD_MS 5U
#define BMI088_CALIBRATION_SAMPLES 400U
#define BMI088_CALIBRATION_TIMEOUT_MS 2500U
#define BMI088_GRAVITY 9.80665f
typedef enum {
    BMI088_STATUS_OK, BMI088_STATUS_NOT_INITIALIZED, BMI088_STATUS_BUSY,
    BMI088_STATUS_ACCEL_IO, BMI088_STATUS_GYRO_IO, BMI088_STATUS_TEMP_IO,
    BMI088_STATUS_BAD_DATA, BMI088_STATUS_ARGUMENT
} BMI088_Status;
typedef struct {
    float gyro[3], accel[3], temperature;
    uint32_t timestamp_ms, sequence;
    BMI088_Status communication_status;
} BMI088_Sample;
typedef struct {
    uint8_t accel_init_error, gyro_init_error, initialized;
    HAL_StatusTypeDef last_spi_status;
    BMI088_Status last_read_status;
    uint32_t read_errors;
} BMI088_Diagnostics;
typedef enum {
    BMI088_CAL_IDLE, BMI088_CAL_REQUESTED, BMI088_CAL_RUNNING,
    BMI088_CAL_SUCCEEDED, BMI088_CAL_FAILED
} BMI088_CalibrationState;
typedef struct {
    BMI088_CalibrationState state;
    uint16_t consecutive_samples;
    uint32_t started_ms, generation;
    float accel_offset[3], gyro_offset[3];
} BMI088_Calibration;
extern float BMI088_ACCEL_SEN, BMI088_GYRO_SEN;
/* Init ONCE after MX_GPIO/SPI init, before MX_FREERTOS_Init/osKernelStart.
 * Runtime SPI access/calibration/attitude update is owned by the BMI088 task.
 * Other tasks use GetSample, RequestCalibration and attitude snapshot APIs. */
uint8_t BMI088_init(void);
/* Runtime entry: acquire one complete sample, process calibration, update
 * g_Attitude. Returns 1 only if the resulting attitude is ready; no init here. */
uint8_t BMI088Read(void);
/* Claims SPI after startup init, without reinitializing either sensor. */
uint8_t BMI088_BindTask(void);
uint8_t bmi088_accel_init(void);
uint8_t bmi088_gyro_init(void);
BMI088_Status BMI088_ReadSample(BMI088_Sample *out);
uint8_t BMI088_read(float gyro[3], float accel[3], float *temperature);
uint8_t BMI088_GetSample(BMI088_Sample *out);
void BMI088_GetDiagnostics(BMI088_Diagnostics *out);
uint8_t BMI088_RequestCalibration(void);
void BMI088_GetCalibration(BMI088_Calibration *out);
void BMI088_CalibrationStep(const BMI088_Sample *sample); /* NULL on read failure. */
#endif
