#ifndef MOTOR6014_TYPES_H
#define MOTOR6014_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR6014_ID_MIN              1U
#define MOTOR6014_ID_MAX              12U
#define MOTOR6014_SLOT_COUNT          13U
#define MOTOR6014_TX_SIZE             20U
#define MOTOR6014_RX_SIZE             26U
#define MOTOR6014_GEAR_RATIO          (38.0f / 3.0f)
#define MOTOR6014_DEFAULT_TIMEOUT_MS  2U

typedef enum {
    MOTOR6014_OK = 0,
    MOTOR6014_BUSY,
    MOTOR6014_INVALID_ARGUMENT,
    MOTOR6014_INVALID_ID,
    MOTOR6014_NOT_INITIALIZED,
    MOTOR6014_OUT_OF_RANGE,
    MOTOR6014_UART_ERROR,
    MOTOR6014_TIMEOUT,
    MOTOR6014_CRC_ERROR,
    MOTOR6014_NO_DATA
} Motor6014_Status_t;

typedef enum {
    MOTOR6014_MODE_STOP = 0,
    MOTOR6014_MODE_FOC = 1
} Motor6014_Mode_t;

/* All commands use rotor-shaft rad, rad/s and N.m. */
typedef struct {
    Motor6014_Mode_t mode;
    float position;
    float velocity;
    float torque;
    float kp; /* Raw gain 0..32767, scale 12800. Negative codes reserved. */
    float kd; /* Raw gain 0..32767, scale 51200. Negative codes reserved. */
    bool timeout_enable; /* Enable the motor's built-in one-second protection. */
} Motor6014_Command_t;

/* Feedback is also expressed at the rotor shaft. */
typedef struct {
    uint8_t id;
    Motor6014_Mode_t mode; /* Preserve all three received mode bits (0..7). */
    bool timeout_triggered;
    float position;
    float velocity;
    float torque;
    int8_t driver_temperature;  /* Degrees Celsius, signed. */
    uint8_t winding_temperature; /* Degrees Celsius, unsigned. */
    float voltage;             /* Volts. */
    uint32_t error;
    uint8_t warning;
    bool valid;
    uint32_t last_update_ms;
    uint32_t sequence;
} Motor6014_Data_t;

typedef struct {
    float position; /* rad */
    float velocity; /* rad/s */
    float torque;   /* N.m */
} Motor6014_MechanicalState_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR6014_TYPES_H */
