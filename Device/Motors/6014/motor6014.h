#ifndef MOTOR6014_H
#define MOTOR6014_H

#include "stm32h7xx_hal.h"
#include "motor6014_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR6014_RX_BUFFER_SIZE 128U

typedef struct {
    Motor6014_Command_t cmd[MOTOR6014_SLOT_COUNT];
    volatile Motor6014_Data_t data[MOTOR6014_SLOT_COUNT];
} Motor6014_State_t;

extern Motor6014_State_t motor6014;

typedef struct {
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t crc_error_count;
    uint32_t invalid_frame_count;
    uint32_t unexpected_reply_count;
    uint32_t timeout_count;
    uint32_t uart_error_count;
} Motor6014_Statistics_t;

/* Caller-owned, static lifetime, DMA-accessible RAM. Treat internals as read-only.
 * This project places static objects in RAM_D1 and does not enable DCache.
 * Do not place this object on the stack or in DTCM. */
typedef struct Motor6014_Port {
    UART_HandleTypeDef *huart;
    struct Motor6014_Port *next;
    uint32_t timeout_ms;
    uint32_t start_ms;
    uint8_t tx_buffer[MOTOR6014_TX_SIZE];
    uint8_t rx_buffer[MOTOR6014_RX_BUFFER_SIZE];
    uint8_t frame[MOTOR6014_RX_SIZE];
    uint16_t rx_position;
    uint16_t frame_length;
    uint8_t pending_id;
    volatile bool busy;
    volatile bool tx_done;
    volatile bool reply_done;
    volatile bool rx_running;
    volatile Motor6014_Status_t last_status;
    volatile uint32_t last_uart_error;
    volatile Motor6014_Statistics_t stats;
} Motor6014_Port_t;

/* Main context with interrupts enabled, once per port/UART, after MX init.
 * Requires RX DMA_CIRCULAR, TX DMA_NORMAL, byte alignment, incrementing memory,
 * fixed peripheral address, correct DMA directions and UART 8N1 TX_RX.
 * timeout_ms must be in [1, INT32_MAX]; default is 2 ms. */
Motor6014_Status_t Motor6014_PortInit(
    Motor6014_Port_t *port, UART_HandleTypeDef *huart, uint32_t timeout_ms);

/* Validate and store one command. This does not start UART transmission.
 * All physical quantities are at the rotor shaft. A successful call records
 * the unquantized arguments in cmd[id], even if a later SendCommand fails. */
Motor6014_Status_t Motor6014_SetCommand(
    UART_HandleTypeDef *huart,
    uint8_t id,
    Motor6014_Mode_t mode,
    float position,
    float velocity,
    float torque,
    float kp,
    float kd,
    bool timeout_enable);

/* Send the command previously stored by SetCommand. OK means DMA accepted
 * the request, not motor acceptance. The call may be made from an ISR when
 * the UART is idle; timeout recovery is deferred to a main-context call. */
Motor6014_Status_t Motor6014_SendCommand(
    UART_HandleTypeDef *huart, uint8_t id);

/* Copy a coherent feedback snapshot. NO_DATA leaves *data unchanged. */
Motor6014_Status_t Motor6014_GetDataSnapshot(uint8_t id, Motor6014_Data_t *data);

/* Forward the corresponding HAL callbacks here. Unknown UARTs are ignored.
 * RX HT/TC events must remain enabled; DMA IRQs must be serviced promptly. */
void Motor6014_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void Motor6014_OnTxComplete(UART_HandleTypeDef *huart);
void Motor6014_OnError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR6014_H */
