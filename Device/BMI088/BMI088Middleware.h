#ifndef BMI088_MIDDLEWARE_H
#define BMI088_MIDDLEWARE_H
#include "main.h"
#define BMI088_SPI_TIMEOUT_MS 1U
void BMI088_GPIO_init(void);
void BMI088_com_init(void);
void BMI088_delay_ms(uint16_t ms);
uint8_t BMI088_delay_us(uint16_t us); /* 0: DWT unavailable/stopped. */
uint8_t BMI088_CycleElapsed(uint32_t start, uint32_t now, uint32_t cycles);
/* SPI owner task only; command, dummy and payload share one CS assertion. */
HAL_StatusTypeDef BMI088_ReadRegisters(uint8_t accel, uint8_t reg, uint8_t *data, uint8_t len);
HAL_StatusTypeDef BMI088_WriteRegister(uint8_t accel, uint8_t reg, uint8_t data);
#endif
