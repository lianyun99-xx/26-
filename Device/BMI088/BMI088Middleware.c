#include "BMI088Middleware.h"
#include "cmsis_os2.h"
#include <string.h>
extern SPI_HandleTypeDef hspi2;
static void select_sensor(uint8_t accel, GPIO_PinState state)
{
    HAL_GPIO_WritePin(accel ? ACC_CS_GPIO_Port : GYRO_CS_GPIO_Port,
                      accel ? ACC_CS_Pin : GYRO_CS_Pin, state);
}
void BMI088_GPIO_init(void)
{
    select_sensor(1, GPIO_PIN_SET);
    select_sensor(0, GPIO_PIN_SET);
}
void BMI088_com_init(void) { /* MX_SPI2_Init owns peripheral configuration. */ }
void BMI088_delay_ms(uint16_t ms)
{
    if (osKernelGetState() == osKernelRunning) {
        uint32_t ticks = ((uint32_t)ms * osKernelGetTickFreq() + 999U) / 1000U;
        if (ticks) (void)osDelay(ticks);
    } else HAL_Delay(ms); /* HAL timebase is TIM4, including before the RTOS. */
}
uint8_t BMI088_CycleElapsed(uint32_t start, uint32_t now, uint32_t cycles)
{
    return (uint32_t)(now - start) >= cycles;
}
uint8_t BMI088_delay_us(uint16_t us)
{
    if (!us) return 1;
    if (SystemCoreClock < 1000000U) return 0;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB(); __ISB();
    if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) return 0;
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = (uint32_t)us * (SystemCoreClock / 1000000U);
    uint32_t last = start, stagnant = 0;
    while (!BMI088_CycleElapsed(start, DWT->CYCCNT, cycles)) {
        uint32_t now = DWT->CYCCNT;
        if (now == last) { if (++stagnant >= 10000U) return 0; }
        else { last = now; stagnant = 0; }
    }
    return 1;
}
HAL_StatusTypeDef BMI088_ReadRegisters(uint8_t accel, uint8_t reg, uint8_t *data, uint8_t len)
{
    if (!data || len == 0 || len > 16) return HAL_ERROR;
    uint8_t tx[18] = {0}, rx[18] = {0};
    uint8_t prefix = accel ? 2 : 1;
    tx[0] = reg | 0x80;
    select_sensor(accel, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, len + prefix, BMI088_SPI_TIMEOUT_MS);
    select_sensor(accel, GPIO_PIN_SET);
    if (status == HAL_OK) memcpy(data, rx + prefix, len);
    return status;
}
HAL_StatusTypeDef BMI088_WriteRegister(uint8_t accel, uint8_t reg, uint8_t data)
{
    uint8_t tx[2], rx[2] = {0};
    tx[0] = reg & 0x7f; tx[1] = data;
    select_sensor(accel, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, BMI088_SPI_TIMEOUT_MS);
    select_sensor(accel, GPIO_PIN_SET);
    return status;
}
