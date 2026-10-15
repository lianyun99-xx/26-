#include "motor6014.h"

#include <limits.h>
#include <string.h>

Motor6014_State_t motor6014;
static Motor6014_Port_t *ports;

static uint32_t Lock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return mask;
}

static void Unlock(uint32_t mask)
{
    __DMB();
    __set_PRIMASK(mask);
}

static Motor6014_Port_t *FindPort(UART_HandleTypeDef *huart)
{
    Motor6014_Port_t *port;
    for (port = ports; port != NULL; port = port->next) {
        if (port->huart == huart) {
            return port;
        }
    }
    return NULL;
}

static bool ValidDMA(const DMA_HandleTypeDef *dma, uint32_t direction, uint32_t mode)
{
    return dma->Init.Direction == direction && dma->Init.Mode == mode
        && dma->Init.MemInc == DMA_MINC_ENABLE
        && dma->Init.PeriphInc == DMA_PINC_DISABLE
        && dma->Init.MemDataAlignment == DMA_MDATAALIGN_BYTE
        && dma->Init.PeriphDataAlignment == DMA_PDATAALIGN_BYTE;
}

static void FinishIfComplete(Motor6014_Port_t *port)
{
    if (port->busy && port->tx_done && port->reply_done) {
        port->busy = false;
        port->pending_id = 0U;
        port->last_status = MOTOR6014_OK;
    }
}

static void AcceptFeedback(Motor6014_Port_t *port, Motor6014_Data_t *data)
{
    data->last_update_ms = HAL_GetTick();
    data->sequence = motor6014.data[data->id].sequence + 1U;
    motor6014.data[data->id] = *data;
    port->stats.rx_count++;
    if (port->busy && port->pending_id == data->id) {
        port->reply_done = true;
        FinishIfComplete(port);
    } else {
        port->stats.unexpected_reply_count++;
    }
}

/* Retain a possible embedded header after a failed candidate, including a
 * trailing single FC. Never discard a whole DMA chunk on CRC failure. */
static void Resynchronize(Motor6014_Port_t *port)
{
    uint16_t offset;
    for (offset = 1U; offset + 1U < port->frame_length; ++offset) {
        if (port->frame[offset] == 0xFCU && port->frame[offset + 1U] == 0xEEU) {
            port->frame_length = (uint16_t)(port->frame_length - offset);
            memmove(port->frame, &port->frame[offset], port->frame_length);
            return;
        }
    }
    if (port->frame[port->frame_length - 1U] == 0xFCU) {
        port->frame[0] = 0xFCU;
        port->frame_length = 1U;
    } else {
        port->frame_length = 0U;
    }
}

static void ParseByte(Motor6014_Port_t *port, uint8_t byte)
{
    Motor6014_Data_t data;
    Motor6014_Status_t status;
    if (port->frame_length == 0U) {
        if (byte == 0xFCU) {
            port->frame[0] = byte;
            port->frame_length = 1U;
        }
        return;
    }
    if (port->frame_length == 1U && byte != 0xEEU) {
        port->frame_length = byte == 0xFCU ? 1U : 0U;
        return;
    }
    port->frame[port->frame_length++] = byte;
    if (port->frame_length != MOTOR6014_RX_SIZE) {
        return;
    }
    status = Motor6014_DecodeFeedback(port->frame, &data);
    if (status == MOTOR6014_OK) {
        AcceptFeedback(port, &data);
        port->frame_length = 0U;
    } else {
        if (status == MOTOR6014_CRC_ERROR) {
            port->stats.crc_error_count++;
        } else {
            port->stats.invalid_frame_count++;
        }
        Resynchronize(port);
    }
}

Motor6014_Status_t Motor6014_PortInit(
    Motor6014_Port_t *port, UART_HandleTypeDef *huart, uint32_t timeout_ms)
{
    Motor6014_Port_t *existing;
    HAL_StatusTypeDef status;
    uint32_t mask;
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U || port == NULL || huart == NULL
        || huart->Instance == NULL || huart->hdmarx == NULL || huart->hdmatx == NULL
        || timeout_ms == 0U || timeout_ms > INT32_MAX
        || !ValidDMA(huart->hdmarx, DMA_PERIPH_TO_MEMORY, DMA_CIRCULAR)
        || !ValidDMA(huart->hdmatx, DMA_MEMORY_TO_PERIPH, DMA_NORMAL)
        || huart->Init.WordLength != UART_WORDLENGTH_8B
        || huart->Init.StopBits != UART_STOPBITS_1
        || huart->Init.Parity != UART_PARITY_NONE
        || huart->Init.Mode != UART_MODE_TX_RX) {
        return MOTOR6014_INVALID_ARGUMENT;
    }
    mask = Lock();
    for (existing = ports; existing != NULL; existing = existing->next) {
        if (existing == port || existing->huart == huart) {
            Unlock(mask);
            return MOTOR6014_INVALID_ARGUMENT;
        }
    }
    memset(port, 0, sizeof(*port));
    port->huart = huart;
    port->timeout_ms = timeout_ms;
    port->next = ports;
    ports = port; /* Register before starting DMA so callbacks can find it. */
    status = HAL_UARTEx_ReceiveToIdle_DMA(huart, port->rx_buffer,
        MOTOR6014_RX_BUFFER_SIZE);
    if (status != HAL_OK) {
        ports = port->next;
        port->huart = NULL;
        port->next = NULL;
        port->last_status = status == HAL_BUSY ? MOTOR6014_BUSY : MOTOR6014_UART_ERROR;
        Unlock(mask);
        /* A failed start can have partly configured RX. Do not abort a UART
         * that returned BUSY before our attempt (it belongs to another user). */
        if (status != HAL_BUSY) {
            (void)HAL_UART_AbortReceive(huart);
        }
        return port->last_status;
    }
    port->rx_running = true;
    port->last_status = MOTOR6014_OK;
    Unlock(mask);
    return MOTOR6014_OK;
}

Motor6014_Status_t Motor6014_SetCommand(
    UART_HandleTypeDef *huart, uint8_t id, Motor6014_Mode_t mode,
    float position, float velocity, float torque, float kp, float kd,
    bool timeout_enable)
{
    Motor6014_Command_t command = {0};
    uint8_t frame[MOTOR6014_TX_SIZE];
    Motor6014_Status_t status;
    uint32_t mask;

    if (id < MOTOR6014_ID_MIN || id > MOTOR6014_ID_MAX) {
        return MOTOR6014_INVALID_ID;
    }
    if (huart == NULL) {
        return MOTOR6014_INVALID_ARGUMENT;
    }
    if (FindPort(huart) == NULL) {
        return MOTOR6014_NOT_INITIALIZED;
    }
    command.mode = mode;
    command.position = position;
    command.velocity = velocity;
    command.torque = torque;
    command.kp = kp;
    command.kd = kd;
    command.timeout_enable = timeout_enable;
    status = Motor6014_EncodeCommand(id, &command, frame);
    if (status != MOTOR6014_OK) {
        return status;
    }
    mask = Lock();
    motor6014.cmd[id] = command;
    Unlock(mask);
    return MOTOR6014_OK;
}

/* The blocking HAL abort functions must never run with the tick disabled or
 * in an ISR. Normal transmission and callbacks do not need this recovery. */
static Motor6014_Status_t PreparePort(Motor6014_Port_t *port)
{
    uint32_t mask;
    HAL_StatusTypeDef status;

    mask = Lock();
    if (port->busy
        && (uint32_t)(HAL_GetTick() - port->start_ms) <= port->timeout_ms) {
        Unlock(mask);
        return MOTOR6014_BUSY;
    }
    if (!port->busy && port->rx_running) {
        Unlock(mask);
        return MOTOR6014_OK;
    }
    Unlock(mask);
    if (__get_IPSR() != 0U || __get_PRIMASK() != 0U) {
        return port->rx_running ? MOTOR6014_BUSY : MOTOR6014_UART_ERROR;
    }

    mask = Lock();
    if (port->busy && port->last_status != MOTOR6014_UART_ERROR) {
        port->last_status = MOTOR6014_TIMEOUT;
        port->stats.timeout_count++;
    }
    Unlock(mask);
    /* Abort only TX on a missing reply/unfinished send; RX circular DMA keeps
     * collecting delayed feedback. An RX hardware fault is repaired below. */
    status = HAL_UART_AbortTransmit(port->huart);
    if (status != HAL_OK) {
        port->last_status = MOTOR6014_UART_ERROR;
        port->last_uart_error = port->huart->ErrorCode;
        return MOTOR6014_UART_ERROR;
    }

    mask = Lock();
    port->busy = false;
    port->pending_id = 0U;
    port->tx_done = false;
    port->reply_done = false;
    Unlock(mask);
    if (!port->rx_running) {
        status = HAL_UART_AbortReceive(port->huart);
        if (status != HAL_OK) {
            port->last_status = MOTOR6014_UART_ERROR;
            port->last_uart_error = port->huart->ErrorCode;
            return MOTOR6014_UART_ERROR;
        }
        mask = Lock();
        port->rx_position = 0U;
        port->frame_length = 0U;
        status = HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->rx_buffer,
            MOTOR6014_RX_BUFFER_SIZE);
        port->rx_running = status == HAL_OK;
        Unlock(mask);
        if (status != HAL_OK) {
            port->last_status = MOTOR6014_UART_ERROR;
            port->last_uart_error = port->huart->ErrorCode;
            return MOTOR6014_UART_ERROR;
        }
    }
    return MOTOR6014_OK;
}

Motor6014_Status_t Motor6014_SendCommand(UART_HandleTypeDef *huart, uint8_t id)
{
    Motor6014_Command_t command;
    uint8_t frame[MOTOR6014_TX_SIZE];
    Motor6014_Port_t *port;
    Motor6014_Port_t *other;
    Motor6014_Status_t status;
    HAL_StatusTypeDef hal_status;
    uint32_t mask;

    if (id < MOTOR6014_ID_MIN || id > MOTOR6014_ID_MAX) {
        return MOTOR6014_INVALID_ID;
    }
    if (huart == NULL) {
        return MOTOR6014_INVALID_ARGUMENT;
    }
    port = FindPort(huart);
    if (port == NULL) {
        return MOTOR6014_NOT_INITIALIZED;
    }
    mask = Lock();
    command = motor6014.cmd[id];
    Unlock(mask);
    status = Motor6014_EncodeCommand(id, &command, frame);
    if (status != MOTOR6014_OK) {
        return status;
    }
    status = PreparePort(port);
    if (status != MOTOR6014_OK) {
        return status;
    }
    mask = Lock();
    if (port->busy || !port->rx_running) {
        Unlock(mask);
        return MOTOR6014_BUSY;
    }
    /* Global IDs must also not have concurrent transactions on two UARTs. */
    for (other = ports; other != NULL; other = other->next) {
        if (other->busy && other->pending_id == id) {
            Unlock(mask);
            return MOTOR6014_BUSY;
        }
    }
    memcpy(port->tx_buffer, frame, sizeof(frame));
    port->busy = true;
    port->pending_id = id;
    port->tx_done = false;
    port->reply_done = false;
    port->start_ms = HAL_GetTick();
    hal_status = HAL_UART_Transmit_DMA(huart, port->tx_buffer, MOTOR6014_TX_SIZE);
    if (hal_status != HAL_OK) {
        port->busy = false;
        port->pending_id = 0U;
        port->last_status = hal_status == HAL_BUSY ? MOTOR6014_BUSY : MOTOR6014_UART_ERROR;
        Unlock(mask);
        return port->last_status;
    }
    port->stats.tx_count++;
    port->last_status = MOTOR6014_BUSY;
    Unlock(mask);
    return MOTOR6014_OK;
}

void Motor6014_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    Motor6014_Port_t *port = FindPort(huart);
    uint32_t remaining;
    uint32_t mask;
    uint16_t position;
    if (port == NULL || size == 0U || size > MOTOR6014_RX_BUFFER_SIZE) {
        return;
    }
    mask = Lock();
    if (!port->rx_running) {
        Unlock(mask);
        return;
    }
    /* Sample the current DMA cursor, not the historical HT/TC Size. A delayed
     * HT interrupt can otherwise follow an IDLE event with a smaller Size and
     * be mistaken for an entire wrap. HT must remain enabled to observe each
     * half-ring; a full ring overwritten during IRQ starvation is unrecoverable. */
    remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > MOTOR6014_RX_BUFFER_SIZE) {
        port->stats.invalid_frame_count++;
        Unlock(mask);
        return;
    }
    position = (uint16_t)(MOTOR6014_RX_BUFFER_SIZE - remaining);
    if (position == MOTOR6014_RX_BUFFER_SIZE) {
        position = 0U;
    }
    __DMB();
    while (port->rx_position != position) {
        ParseByte(port, port->rx_buffer[port->rx_position]);
        port->rx_position++;
        if (port->rx_position == MOTOR6014_RX_BUFFER_SIZE) {
            port->rx_position = 0U;
        }
    }
    Unlock(mask);
}

void Motor6014_OnTxComplete(UART_HandleTypeDef *huart)
{
    Motor6014_Port_t *port = FindPort(huart);
    uint32_t mask;
    if (port == NULL) {
        return;
    }
    mask = Lock();
    if (port->busy) {
        port->tx_done = true;
        FinishIfComplete(port);
    }
    Unlock(mask);
}

void Motor6014_OnError(UART_HandleTypeDef *huart)
{
    Motor6014_Port_t *port = FindPort(huart);
    uint32_t mask;
    if (port == NULL) {
        return;
    }
    mask = Lock();
    port->stats.uart_error_count++;
    port->last_uart_error = huart->ErrorCode;
    port->last_status = MOTOR6014_UART_ERROR;
    port->rx_running = false;
    port->busy = false;
    port->pending_id = 0U;
    /* The HAL error callback can run at priority zero. Mark for repair by the
     * next main-context SendCommand instead of blocking here. */
    port->tx_done = false;
    port->reply_done = false;
    Unlock(mask);
}

Motor6014_Status_t Motor6014_GetDataSnapshot(uint8_t id, Motor6014_Data_t *data)
{
    uint32_t mask;
    if (id < MOTOR6014_ID_MIN || id > MOTOR6014_ID_MAX) {
        return MOTOR6014_INVALID_ID;
    }
    if (data == NULL) {
        return MOTOR6014_INVALID_ARGUMENT;
    }
    mask = Lock();
    if (!motor6014.data[id].valid) {
        Unlock(mask);
        return MOTOR6014_NO_DATA;
    }
    *data = motor6014.data[id];
    Unlock(mask);
    return MOTOR6014_OK;
}
