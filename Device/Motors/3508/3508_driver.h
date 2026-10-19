#ifndef D3508_DRIVER_H
#define D3508_DRIVER_H
#include "main.h"
#define D3508_REDUCTION_RATIO 15.7f
#define D3508_ENCODER_RES 8192.0f
#define D3508_RAD_PER_TICK (6.28318530718f / (D3508_ENCODER_RES * D3508_REDUCTION_RATIO))
#define D3508_RPM_TO_WHEEL_RADPS (6.28318530718f / (60.0f * D3508_REDUCTION_RATIO))
#define D3508_DEFAULT_DT 0.005f
#define D3508_DEFAULT_TIMEOUT_MS 20U
/* MIT output limits: edit these instead of adding limit arguments to Set. */
#define D3508_CURRENT_LIMIT_UNITS 16000
#if D3508_CURRENT_LIMIT_UNITS < 0 || D3508_CURRENT_LIMIT_UNITS > 16384
#error "D3508_CURRENT_LIMIT_UNITS must fit the C620 current range [0, 16384]"
#endif
#define D3508_CURRENT_UNITS_PER_AMP 819.2f
#define D3508_ROTOR_TORQUE_CONSTANT 0.0156f /* Nm/A */
#define D3508_MECHANICAL_POWER_LIMIT_W 162.0f /* Per motor, estimated. */
#define D3508_MIT_MAX_ROTOR_RPM 6000.0f
#define D3508_MIT_MAX_WHEEL_TORQUE_NM \
    (D3508_CURRENT_LIMIT_UNITS / D3508_CURRENT_UNITS_PER_AMP * \
     D3508_ROTOR_TORQUE_CONSTANT * D3508_REDUCTION_RATIO)
typedef enum {
    MOTOR3508_OK, MOTOR3508_BUSY, MOTOR3508_INVALID_ARGUMENT,
    MOTOR3508_INVALID_ID, MOTOR3508_NOT_READY, MOTOR3508_DISABLED,
    MOTOR3508_FEEDBACK_TIMEOUT, MOTOR3508_BUS_FAULT, MOTOR3508_CAN_ERROR
} Motor3508_Status;
typedef struct {
    int64_t position; /* Rotor encoder ticks, 8192 per rotor turn. */
    float velocity;   /* Rotor RPM. */
    float torque;     /* Wheel feedforward torque, Nm. */
    float kp, kd;     /* Wheel Nm/rad and Nm/(rad/s). */
} Motor3508_Command;
typedef struct {
    Motor3508_Command cmd[5]; /* Physical C620 ID 1..4; slot 0 unused. */
    volatile Motor3508_Status Set_status_port1, Set_status_port2;
    volatile Motor3508_Status Send_status_port1, Send_status_port2;
    volatile Motor3508_Status last_set_status, last_send_status;
} Motor3508_State;
extern Motor3508_State motor3508;
/* Like Motor6014: Set calculates/stores, Send submits the stored currents.
 * Physical IDs: FDCAN1 1/2, FDCAN2 3/4. Units retain the existing wheel MIT API.
 * Call Set for both motors, then Send ONCE per bus (C620 uses a group frame).
 * These calls belong to the same application/CAN task, not to an ISR. */
void Motor3508_SetCommand(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                        int64_t position, float velocity, float torque, float kp, float kd);
void Motor3508_SendCommand(FDCAN_HandleTypeDef *hfdcan);
Motor3508_Status Motor3508_GetCommandSnapshot(FDCAN_HandleTypeDef *hfdcan, uint8_t id,
                                             Motor3508_Command *out);
typedef enum {
    D3508_NOT_READY, D3508_DISABLED, D3508_ENABLED,
    D3508_FEEDBACK_TIMEOUT, D3508_BUS_FAULT
} D3508_State;
typedef struct {
    uint16_t cur_angle;
    int16_t cur_speed;
    float filter_speed;
    int16_t actual_current;
    uint8_t temperature;
    uint16_t last_angle;
    int64_t total_angle;
    int32_t total_round;
    int64_t target_angle;
    float target_speed;
    int16_t Out_Current;
    float speed_err_sum, position_err_sum, torque_err_sum, err_last;
    uint8_t init_flag;
    uint8_t target_valid; /* Cleared on disable/fault/enable; submit a new target. */
    uint32_t last_update_ms;
    D3508_State state;
    uint32_t epoch;
} Motor_3508_T;
typedef struct { float Kp, Ki, Kd, Max_Out, Max_Sum; } PID_3508_T;
typedef struct { Motor_3508_T motor; uint32_t feedback_age_ms; } D3508_Feedback;
typedef struct {
    HAL_StatusTypeDef last_tx_status;
    uint32_t tx_errors, fifo_full_count, bus_off_count, rx_rejected;
    uint8_t fault_latched, recovery_pending;
} D3508_BusStatus;
typedef struct { HAL_StatusTypeDef bus1, bus2; } D3508_SendResult;
/* Compatibility globals: read snapshots and use APIs for writes. PID tuning
 * and synchronous calculations must have one application task as owner. */
extern Motor_3508_T Motors[4];
extern PID_3508_T Speed_PID, Position_PID;
/* Startup only, before CAN notifications. Arms IDs 1..4 for automatic enable
 * on first valid feedback; outputs stay zero until a fresh Set command.
 * Disable/timeout/bus-off cancels automatic enable; recovery is explicit. */
void D3508_Init(void);
void FDCAN1_Filter_Init(void);
void FDCAN2_Filter_Init(void);
void FDCAN3_Filter_Init(void); /* Deprecated alias for the old bus2 API. */
uint8_t D3508_Enable(int i);
void D3508_Disable(int i);
uint8_t D3508_SetFeedbackTimeout(uint32_t timeout_ms);
uint8_t D3508_GetFeedback(int i, D3508_Feedback *out);
uint8_t D3508_GetBusStatus(unsigned bus, D3508_BusStatus *out);
uint8_t D3508_RequestBusRecovery(unsigned bus); /* Serviced by the CAN task. */
void D3508_OnBusOff(FDCAN_HandleTypeDef *h);
void D3508_Maintain(void); /* CAN owner task only. */
D3508_SendResult D3508_SendCurrent(void); /* Submission status, not CAN ACK. */
void send_current(void);
/* Weak application hook called before each 200 Hz submission. */
void D3508_ApplicationControl(float dt);
uint8_t D3508_Decode(FDCAN_HandleTypeDef *h, const FDCAN_RxHeaderTypeDef *header,
                     const uint8_t data[8]);
void Clear_Motor_PID(int i);
uint8_t D3508_SetSpeedTarget(int i, float rotor_rpm);
void PID_Calc_Speed(int i);
void PID_Calc_SpeedDt(int i, float dt);
void PID_Calc_Position(int i, float rotor_ticks);
void PID_Calc_PositionDt(int i, int64_t rotor_ticks, float dt);
void PID_Calc_Torque(int i, float rotor_nm);
void MIT_Wheel_Control(int i, int64_t rotor_ticks, float rotor_rpm,
                       float kp, float kd, float wheel_ff_nm, float wheel_limit_nm);
/* Per-motor mechanical estimate (162 W), not an electrical input power limit. */
int16_t Power_Limit(float current_units, float rotor_rpm);
#endif
