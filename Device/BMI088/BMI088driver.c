#include "BMI088driver.h"
#include "driver_sync.h"
#include "cmsis_os2.h"
#include <math.h>
#include <string.h>
float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;
static BMI088_Sample published;
static uint8_t have_sample;
static BMI088_Diagnostics diagnostics;
static BMI088_Calibration calibration;
static osThreadId_t spi_owner;
static float sums[6];
static uint32_t calibration_sequence;
static uint8_t calibration_have_sequence;
static uint8_t owner_ok(void)
{
    return osKernelGetState() != osKernelRunning || (spi_owner && osThreadGetId() == spi_owner);
}
uint8_t BMI088_BindTask(void)
{
    if (osKernelGetState() != osKernelRunning) return 0;
    osThreadId_t caller = osThreadGetId();
    if (!caller) return 0;
    uint32_t key = Driver_Lock();
    uint8_t ok = !spi_owner || spi_owner == caller;
    if (ok) spi_owner = caller;
    Driver_Unlock(key);
    return ok;
}
static const uint8_t accel_config[][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, 0, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, 0, BMI088_INT_MAP_DATA_ERROR}
};
static const uint8_t gyro_config[][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, 0, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, 0, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, 0, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};
static uint8_t sensor_init(uint8_t accel)
{
    if (!owner_ok()) return BMI088_FATAL_ERROR;
    uint8_t id = 0;
    uint8_t reset_reg = accel ? BMI088_ACC_SOFTRESET : BMI088_GYRO_SOFTRESET;
    uint8_t expected = accel ? BMI088_ACC_CHIP_ID_VALUE : BMI088_GYRO_CHIP_ID_VALUE;
    /* First accelerometer CS transaction selects SPI; ignore its ID value. */
    if (BMI088_ReadRegisters(accel, 0, &id, 1) != HAL_OK ||
        !BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME) ||
        BMI088_WriteRegister(accel, reset_reg, 0xB6) != HAL_OK) return BMI088_FATAL_ERROR;
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    if (BMI088_ReadRegisters(accel, 0, &id, 1) != HAL_OK ||
        !BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME) ||
        BMI088_ReadRegisters(accel, 0, &id, 1) != HAL_OK) return BMI088_FATAL_ERROR;
    if (id != expected) return BMI088_NO_SENSOR;
    const uint8_t (*config)[3] = accel ? accel_config : gyro_config;
    for (unsigned i = 0; i < 6; ++i) {
        if (BMI088_WriteRegister(accel, config[i][0], config[i][1]) != HAL_OK ||
            !BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME) ||
            BMI088_ReadRegisters(accel, config[i][0], &id, 1) != HAL_OK ||
            !BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME)) return BMI088_FATAL_ERROR;
        if (id != config[i][1]) return config[i][2];
    }
    return BMI088_NO_ERROR;
}
uint8_t bmi088_accel_init(void) { return sensor_init(1); }
uint8_t bmi088_gyro_init(void) { return sensor_init(0); }
uint8_t BMI088_init(void)
{
    osThreadId_t caller = osKernelGetState() == osKernelRunning ? osThreadGetId() : NULL;
    if (spi_owner && !owner_ok()) return BMI088_FATAL_ERROR;
    spi_owner = caller;
    BMI088_Diagnostics d = {0};
    d.last_read_status = BMI088_STATUS_NOT_INITIALIZED;
    uint32_t key = Driver_Lock();
    diagnostics = d; have_sample = 0;
    memset(&calibration, 0, sizeof(calibration));
    Driver_Unlock(key);
    BMI088_GPIO_init(); BMI088_com_init();
    d.accel_init_error = bmi088_accel_init();
    d.gyro_init_error = bmi088_gyro_init();
    d.initialized = d.accel_init_error == 0 && d.gyro_init_error == 0;
    key = Driver_Lock(); diagnostics = d; Driver_Unlock(key);
    /* Legacy return: first error; both errors remain separately queryable. */
    return d.accel_init_error ? d.accel_init_error : d.gyro_init_error;
}
static BMI088_Status failure(BMI088_Status status, HAL_StatusTypeDef spi)
{
    uint32_t key = Driver_Lock();
    diagnostics.last_read_status = status; diagnostics.last_spi_status = spi;
    ++diagnostics.read_errors;
    Driver_Unlock(key);
    return status;
}
static int16_t le16(const uint8_t *b) { return (int16_t)(((uint16_t)b[1] << 8) | b[0]); }
BMI088_Status BMI088_ReadSample(BMI088_Sample *out)
{
    if (!out) return BMI088_STATUS_ARGUMENT;
    if (!owner_ok()) return BMI088_STATUS_BUSY;
    BMI088_Diagnostics diag; BMI088_GetDiagnostics(&diag);
    if (!diag.initialized) return BMI088_STATUS_NOT_INITIALIZED;
    uint8_t acc[6], gyro[8], temp[2];
    HAL_StatusTypeDef io = BMI088_ReadRegisters(1, BMI088_ACCEL_XOUT_L, acc, 6);
    if (io != HAL_OK) return failure(BMI088_STATUS_ACCEL_IO, io);
    io = BMI088_ReadRegisters(0, BMI088_GYRO_CHIP_ID, gyro, 8);
    if (io != HAL_OK) return failure(BMI088_STATUS_GYRO_IO, io);
    if (gyro[0] != BMI088_GYRO_CHIP_ID_VALUE) return failure(BMI088_STATUS_BAD_DATA, HAL_OK);
    io = BMI088_ReadRegisters(1, BMI088_TEMP_M, temp, 2);
    if (io != HAL_OK) return failure(BMI088_STATUS_TEMP_IO, io);
    BMI088_Sample s = {0};
    for (unsigned i = 0; i < 3; ++i) {
        s.accel[i] = le16(acc + 2 * i) * BMI088_ACCEL_SEN;
        s.gyro[i] = le16(gyro + 2 + 2 * i) * BMI088_GYRO_SEN;
        if (!isfinite(s.accel[i]) || !isfinite(s.gyro[i])) return failure(BMI088_STATUS_BAD_DATA, HAL_OK);
    }
    int16_t raw_temp = ((uint16_t)temp[0] << 3) | (temp[1] >> 5);
    if (raw_temp > 1023) raw_temp -= 2048;
    s.temperature = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    s.timestamp_ms = HAL_GetTick(); s.communication_status = BMI088_STATUS_OK;
    uint32_t key = Driver_Lock();
    s.sequence = have_sample ? published.sequence + 1U : 1U;
    published = s; have_sample = 1;
    diagnostics.last_spi_status = HAL_OK; diagnostics.last_read_status = BMI088_STATUS_OK;
    Driver_Unlock(key);
    *out = s;
    return BMI088_STATUS_OK;
}
uint8_t BMI088_read(float gyro[3], float accel[3], float *temperature)
{
    if (!gyro || !accel || !temperature) return 0;
    BMI088_Sample s;
    if (BMI088_ReadSample(&s) != BMI088_STATUS_OK) return 0;
    memcpy(gyro, s.gyro, sizeof(s.gyro)); memcpy(accel, s.accel, sizeof(s.accel));
    *temperature = s.temperature;
    return 1;
}
uint8_t BMI088_GetSample(BMI088_Sample *out)
{
    if (!out) return 0;
    uint32_t key = Driver_Lock();
    uint8_t ok = have_sample;
    if (ok) *out = published;
    Driver_Unlock(key); return ok;
}
void BMI088_GetDiagnostics(BMI088_Diagnostics *out)
{
    if (!out) return;
    uint32_t key = Driver_Lock(); *out = diagnostics; Driver_Unlock(key);
}
uint8_t BMI088_RequestCalibration(void)
{
    uint32_t key = Driver_Lock();
    uint8_t ok = diagnostics.initialized && calibration.state != BMI088_CAL_RUNNING &&
                 calibration.state != BMI088_CAL_REQUESTED;
    if (ok) {
        calibration.state = BMI088_CAL_REQUESTED;
        calibration.consecutive_samples = 0;
        ++calibration.generation;
    }
    Driver_Unlock(key); return ok;
}
void BMI088_GetCalibration(BMI088_Calibration *out)
{
    if (!out) return;
    uint32_t key = Driver_Lock(); *out = calibration; Driver_Unlock(key);
}
void BMI088_CalibrationStep(const BMI088_Sample *s)
{
    if (!owner_ok()) return;
    BMI088_Calibration c; BMI088_GetCalibration(&c);
    uint32_t now = HAL_GetTick();
    if (c.state == BMI088_CAL_REQUESTED) {
        c.state = BMI088_CAL_RUNNING; c.started_ms = now;
        c.consecutive_samples = 0;
        memset(sums, 0, sizeof(sums)); calibration_have_sequence = 0;
    }
    if (c.state != BMI088_CAL_RUNNING) return;
    if ((uint32_t)(now - c.started_ms) >= BMI088_CALIBRATION_TIMEOUT_MS) c.state = BMI088_CAL_FAILED;
    else if (!s || s->communication_status != BMI088_STATUS_OK || (uint32_t)(now - s->timestamp_ms) > 5U) {
        c.consecutive_samples = 0; memset(sums, 0, sizeof(sums));
    } else if (!calibration_have_sequence || s->sequence != calibration_sequence) {
        calibration_sequence = s->sequence; calibration_have_sequence = 1;
        float a2 = 0, g2 = 0;
        for (unsigned i = 0; i < 3; ++i) { a2 += s->accel[i] * s->accel[i]; g2 += s->gyro[i] * s->gyro[i]; }
        float g = BMI088_GRAVITY;
        if (!isfinite(a2) || !isfinite(g2) || g2 >= 0.05f * 0.05f ||
            a2 < (0.95f*g)*(0.95f*g) || a2 > (1.05f*g)*(1.05f*g) ||
            fabsf(s->accel[0]) >= 0.05f*g || fabsf(s->accel[1]) >= 0.05f*g || s->accel[2] <= 0) {
            c.consecutive_samples = 0; memset(sums, 0, sizeof(sums));
        } else {
            for (unsigned i = 0; i < 3; ++i) { sums[i] += s->accel[i]; sums[i+3] += s->gyro[i]; }
            if (++c.consecutive_samples == BMI088_CALIBRATION_SAMPLES) {
                for (unsigned i = 0; i < 3; ++i) {
                    c.accel_offset[i] = sums[i] / BMI088_CALIBRATION_SAMPLES;
                    c.gyro_offset[i] = sums[i+3] / BMI088_CALIBRATION_SAMPLES;
                }
                c.accel_offset[2] -= BMI088_GRAVITY;
                c.state = BMI088_CAL_SUCCEEDED;
            }
        }
    }
    uint32_t key = Driver_Lock(); calibration = c; Driver_Unlock(key);
}
