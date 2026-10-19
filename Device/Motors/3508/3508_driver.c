#include "3508_driver.h"
#include "driver_sync.h"
#include <math.h>
#include <string.h>
#include <limits.h>
extern FDCAN_HandleTypeDef hfdcan1, hfdcan2;
Motor_3508_T Motors[4];
Motor3508_State motor3508;
PID_3508_T Speed_PID = {3.0f, 0.3f, 0.0f, 16000.0f, 10000.0f};
PID_3508_T Position_PID = {1.0f, 0.03f, 0.15f, 6000.0f, 0.0f};
static D3508_BusStatus buses[2];
static uint8_t flush_pending[2];
/* One startup request per motor, consumed by its first valid feedback. */
static uint8_t startup_enable_pending[4];
static uint32_t feedback_timeout = D3508_DEFAULT_TIMEOUT_MS;
static int valid(int i) { return i >= 0 && i < 4; }
static int bus_index(const FDCAN_HandleTypeDef *h)
{
    if (!h) return -1;
    if (h->Instance == FDCAN1) return 0;
    if (h->Instance == FDCAN2) return 1;
    return -1;
}
static FDCAN_HandleTypeDef *handle(unsigned b) { return b ? &hfdcan2 : &hfdcan1; }
static float clip(float x, float limit) { return fmaxf(-limit, fminf(x, limit)); }
static void clear_control(Motor_3508_T *m)
{
    m->speed_err_sum = m->position_err_sum = m->torque_err_sum = m->err_last = 0.0f;
    m->target_speed = 0.0f;
    m->target_angle = 0;
    m->target_valid = 0;
    m->Out_Current = 0;
    ++m->epoch;
}
/* Caller holds the state lock; no I/O or floating point arithmetic here. */
static void expire(int i, uint32_t now)
{
    Motor_3508_T *m = &Motors[i];
    if (m->init_flag && (uint32_t)(now - m->last_update_ms) >= feedback_timeout) {
        startup_enable_pending[i] = 0;
        clear_control(m);
        m->init_flag = 0;
        m->state = buses[i / 2].fault_latched ? D3508_BUS_FAULT : D3508_FEEDBACK_TIMEOUT;
        flush_pending[i / 2] = 1;
    }
}
void D3508_Init(void)
{
    uint32_t key = Driver_Lock();
    memset(Motors, 0, sizeof(Motors));
    memset(&motor3508, 0, sizeof(motor3508));
    motor3508.Set_status_port1 = motor3508.Set_status_port2 = MOTOR3508_NOT_READY;
    motor3508.Send_status_port1 = motor3508.Send_status_port2 = MOTOR3508_NOT_READY;
    motor3508.last_set_status = motor3508.last_send_status = MOTOR3508_NOT_READY;
    memset(buses, 0, sizeof(buses));
    buses[0].last_tx_status = buses[1].last_tx_status = HAL_BUSY;
    memset(flush_pending, 0, sizeof(flush_pending));
    /* FDCAN1: IDs 1/2; FDCAN2: IDs 3/4. Enable after feedback is ready. */
    memset(startup_enable_pending, 1, sizeof(startup_enable_pending));
    feedback_timeout = D3508_DEFAULT_TIMEOUT_MS;
    Driver_Unlock(key);
}
void Clear_Motor_PID(int i)
{
    if (!valid(i)) return;
    uint32_t key = Driver_Lock();
    clear_control(&Motors[i]);
    flush_pending[i / 2] = 1;
    Driver_Unlock(key);
}
void D3508_Disable(int i)
{
    if (!valid(i)) return;
    uint32_t key = Driver_Lock();
    startup_enable_pending[i] = 0;
    clear_control(&Motors[i]);
    Motors[i].state = buses[i / 2].fault_latched ? D3508_BUS_FAULT :
                      (Motors[i].init_flag ? D3508_DISABLED : D3508_NOT_READY);
    flush_pending[i / 2] = 1;
    Driver_Unlock(key);
}
uint8_t D3508_Enable(int i)
{
    if (!valid(i)) return 0;
    uint32_t key = Driver_Lock(), now = HAL_GetTick();
    expire(i, now);
    uint8_t ok = Motors[i].init_flag && !buses[i / 2].fault_latched;
    if (ok && Motors[i].state != D3508_ENABLED) {
        clear_control(&Motors[i]);
        Motors[i].state = D3508_ENABLED;
    }
    Driver_Unlock(key);
    return ok;
}
uint8_t D3508_SetFeedbackTimeout(uint32_t ms)
{
    if (ms == 0 || ms > 1000) return 0;
    uint32_t key = Driver_Lock(); feedback_timeout = ms; Driver_Unlock(key);
    return 1;
}
uint8_t D3508_GetFeedback(int i, D3508_Feedback *out)
{
    if (!valid(i) || !out) return 0;
    uint32_t key = Driver_Lock(), now = HAL_GetTick();
    expire(i, now);
    out->motor = Motors[i];
    Driver_Unlock(key);
    out->feedback_age_ms = out->motor.init_flag || out->motor.state == D3508_FEEDBACK_TIMEOUT ?
                            (uint32_t)(now - out->motor.last_update_ms) : UINT32_MAX;
    return 1;
}
uint8_t D3508_GetBusStatus(unsigned bus, D3508_BusStatus *out)
{
    if (bus < 1 || bus > 2 || !out) return 0;
    uint32_t key = Driver_Lock(); *out = buses[bus - 1]; Driver_Unlock(key);
    return 1;
}
void D3508_OnBusOff(FDCAN_HandleTypeDef *h)
{
    int b = bus_index(h);
    if (b < 0) return;
    uint32_t key = Driver_Lock();
    if (!buses[b].fault_latched) ++buses[b].bus_off_count;
    buses[b].fault_latched = 1;
    flush_pending[b] = 1;
    for (int i = b * 2; i < b * 2 + 2; ++i) {
        startup_enable_pending[i] = 0;
        clear_control(&Motors[i]);
        Motors[i].init_flag = 0;
        Motors[i].state = D3508_BUS_FAULT;
    }
    Driver_Unlock(key);
}
static HAL_StatusTypeDef configure(unsigned b)
{
    FDCAN_HandleTypeDef *h = handle(b);
    FDCAN_FilterTypeDef f = {0};
    f.IdType = FDCAN_STANDARD_ID; f.FilterType = FDCAN_FILTER_DUAL;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1 = b ? 0x203 : 0x201; f.FilterID2 = b ? 0x204 : 0x202;
    if (HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_ConfigFilter(h, &f) != HAL_OK || HAL_FDCAN_Start(h) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(h, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0) != HAL_OK)
        return HAL_ERROR;
    return HAL_OK;
}
void FDCAN1_Filter_Init(void) { if (configure(0) != HAL_OK) D3508_OnBusOff(&hfdcan1); }
void FDCAN2_Filter_Init(void) { if (configure(1) != HAL_OK) D3508_OnBusOff(&hfdcan2); }
void FDCAN3_Filter_Init(void) { FDCAN2_Filter_Init(); }
uint8_t D3508_RequestBusRecovery(unsigned bus)
{
    if (bus < 1 || bus > 2) return 0;
    uint32_t key = Driver_Lock();
    uint8_t ok = buses[bus - 1].fault_latched;
    if (ok) buses[bus - 1].recovery_pending = 1;
    Driver_Unlock(key);
    return ok;
}
void D3508_Maintain(void)
{
    for (unsigned b = 0; b < 2; ++b) {
        FDCAN_ProtocolStatusTypeDef p = {0};
        if (HAL_FDCAN_GetProtocolStatus(handle(b), &p) != HAL_OK || p.BusOff)
            D3508_OnBusOff(handle(b));
        uint32_t key = Driver_Lock();
        uint8_t recover = buses[b].recovery_pending;
        buses[b].recovery_pending = 0;
        uint8_t flush = flush_pending[b]; flush_pending[b] = 0;
        Driver_Unlock(key);
        if (flush) (void)HAL_FDCAN_AbortTxRequest(handle(b), FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1);
        if (recover) {
            /* Stop/init/start restarts bus-off recovery without old RX/TX. */
            HAL_StatusTypeDef result = HAL_FDCAN_Stop(handle(b));
            if (result == HAL_OK) result = HAL_FDCAN_Init(handle(b));
            if (result == HAL_OK) result = configure(b);
            if (result == HAL_OK) {
                key = Driver_Lock();
                buses[b].fault_latched = 0;
                for (unsigned i = b * 2; i < b * 2 + 2; ++i) {
                    clear_control(&Motors[i]);
                    Motors[i].init_flag = 0;
                    Motors[i].state = D3508_NOT_READY;
                }
                Driver_Unlock(key);
            }
        }
    }
    uint32_t key = Driver_Lock(), now = HAL_GetTick();
    for (int i = 0; i < 4; ++i) expire(i, now);
    Driver_Unlock(key);
}
uint8_t D3508_Decode(FDCAN_HandleTypeDef *h, const FDCAN_RxHeaderTypeDef *r, const uint8_t d[8])
{
    int b = bus_index(h);
    if (b < 0) return 0;
    if (!r || !d || r->IdType != FDCAN_STANDARD_ID || r->RxFrameType != FDCAN_DATA_FRAME ||
        r->FDFormat != FDCAN_CLASSIC_CAN || r->DataLength != FDCAN_DLC_BYTES_8 ||
        r->Identifier < (b ? 0x203U : 0x201U) || r->Identifier > (b ? 0x204U : 0x202U) ||
        (((uint16_t)d[0] << 8) | d[1]) > 8191U) {
        uint32_t key = Driver_Lock(); ++buses[b].rx_rejected; Driver_Unlock(key);
        return 0;
    }
    int i = (int)r->Identifier - 0x201;
    Motor_3508_T m;
    uint32_t key = Driver_Lock(), now = HAL_GetTick();
    expire(i, now); m = Motors[i];
    uint8_t fault = buses[b].fault_latched;
    Driver_Unlock(key);
    if (fault) return 0;
    uint16_t angle = ((uint16_t)d[0] << 8) | d[1];
    int16_t speed = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
    if (!m.init_flag) {
        m.total_round = 0; m.total_angle = angle; m.filter_speed = speed;
    } else {
        int delta = (int)angle - m.cur_angle;
        if ((delta > 4096 && m.total_round == INT32_MIN) ||
            (delta < -4096 && m.total_round == INT32_MAX)) {
            D3508_Disable(i); return 0;
        }
        if (delta > 4096) --m.total_round;
        if (delta < -4096) ++m.total_round;
        m.total_angle = (int64_t)m.total_round * 8192 + angle;
        m.filter_speed = 0.3f * speed + 0.7f * m.filter_speed;
    }
    key = Driver_Lock();
    Motor_3508_T *dst = &Motors[i];
    if (dst->epoch == m.epoch && !buses[b].fault_latched) {
        dst->last_angle = m.init_flag ? m.cur_angle : angle;
        dst->cur_angle = angle; dst->cur_speed = speed;
        dst->actual_current = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
        dst->temperature = d[6]; dst->total_round = m.total_round;
        dst->total_angle = m.total_angle; dst->filter_speed = m.filter_speed;
        dst->last_update_ms = now; dst->init_flag = 1;
        if (!m.init_flag) {
            dst->state = startup_enable_pending[i] ? D3508_ENABLED : D3508_DISABLED;
            startup_enable_pending[i] = 0;
        }
    }
    Driver_Unlock(key);
    return 1;
}
static uint8_t control_snapshot(int i, Motor_3508_T *m)
{
    D3508_Feedback f;
    if (!D3508_GetFeedback(i, &f) || f.motor.state != D3508_ENABLED) return 0;
    *m = f.motor; return 1;
}
static void commit(int i, const Motor_3508_T *m)
{
    uint32_t key = Driver_Lock(), now = HAL_GetTick();
    expire(i, now);
    Motor_3508_T *dst = &Motors[i];
    if (dst->state == D3508_ENABLED && dst->epoch == m->epoch) {
        dst->target_angle = m->target_angle; dst->target_speed = m->target_speed;
        dst->target_valid = m->target_valid;
        dst->speed_err_sum = m->speed_err_sum; dst->position_err_sum = m->position_err_sum;
        dst->torque_err_sum = m->torque_err_sum; dst->err_last = m->err_last;
        dst->Out_Current = m->Out_Current;
    }
    Driver_Unlock(key);
}
static int good_dt(float dt) { return isfinite(dt) && dt > 0.0f && dt <= 0.1f; }
static int good_pid(PID_3508_T p)
{
    return isfinite(p.Kp) && isfinite(p.Ki) && isfinite(p.Kd) &&
           isfinite(p.Max_Out) && p.Max_Out >= 0 && isfinite(p.Max_Sum) && p.Max_Sum >= 0;
}
/* Integer-domain difference also handles the two INT64 endpoints. */
static float tick_error(int64_t target, int64_t actual)
{
    if (target >= actual) return (float)((uint64_t)target - (uint64_t)actual);
    return -(float)((uint64_t)actual - (uint64_t)target);
}
int16_t Power_Limit(float current, float rpm)
{
    if (!isfinite(current) || !isfinite(rpm)) return 0;
    float w = fmaxf(fabsf(rpm) * 0.10471975512f, 0.5f);
    float limit = fminf(D3508_CURRENT_LIMIT_UNITS,
        (D3508_MECHANICAL_POWER_LIMIT_W / w / D3508_ROTOR_TORQUE_CONSTANT) * D3508_CURRENT_UNITS_PER_AMP);
    return (int16_t)clip(current, limit);
}
static int speed_calc(Motor_3508_T *m, float dt)
{
    PID_3508_T p = Speed_PID;
    if (!good_pid(p) || !isfinite(m->target_speed) || !isfinite(m->filter_speed)) return 0;
    float error = m->target_speed - m->cur_speed;
    /* Keep legacy error-per-cycle sum units, with elapsed-time weighting. */
    m->speed_err_sum = p.Max_Sum > 0 ? clip(m->speed_err_sum + error * dt / D3508_DEFAULT_DT, p.Max_Sum) : 0;
    float derivative = (error - m->err_last) / dt;
    float output = p.Kp * error + p.Ki * m->speed_err_sum * D3508_DEFAULT_DT + p.Kd * derivative;
    m->err_last = error;
    if (!isfinite(output)) return 0;
    m->Out_Current = Power_Limit(clip(output, p.Max_Out), m->filter_speed);
    return 1;
}
uint8_t D3508_SetSpeedTarget(int i, float rpm)
{
    if (!valid(i) || !isfinite(rpm)) { D3508_Disable(i); return 0; }
    Motor_3508_T m;
    if (!control_snapshot(i, &m)) return 0;
    m.target_speed = rpm; m.target_valid = 1; commit(i, &m); return 1;
}
void PID_Calc_SpeedDt(int i, float dt)
{
    if (!valid(i)) return;
    Motor_3508_T m;
    if (!good_dt(dt)) { D3508_Disable(i); return; }
    if (!control_snapshot(i, &m)) return;
    if (!m.target_valid) return;
    if (!speed_calc(&m, dt)) { D3508_Disable(i); return; }
    commit(i, &m);
}
void PID_Calc_Speed(int i) { PID_Calc_SpeedDt(i, D3508_DEFAULT_DT); }
void PID_Calc_PositionDt(int i, int64_t ticks, float dt)
{
    if (!valid(i)) return;
    Motor_3508_T m; PID_3508_T p = Position_PID;
    if (!good_dt(dt) || !good_pid(p)) { D3508_Disable(i); return; }
    if (!control_snapshot(i, &m)) return;
    m.target_angle = ticks;
    m.target_valid = 1;
    float error = tick_error(ticks, m.total_angle);
    if (fabsf(error) < 25.0f) {
        m.target_speed = m.position_err_sum = m.speed_err_sum = m.err_last = 0;
        m.Out_Current = 0;
    } else {
        m.position_err_sum = p.Max_Sum > 0 ? clip(m.position_err_sum + error * dt / D3508_DEFAULT_DT, p.Max_Sum) : 0;
        float rpm = p.Kp * error + p.Ki * m.position_err_sum * D3508_DEFAULT_DT - p.Kd * m.filter_speed;
        if (!isfinite(rpm)) { D3508_Disable(i); return; }
        m.target_speed = clip(rpm, p.Max_Out);
        if (!speed_calc(&m, dt)) { D3508_Disable(i); return; }
    }
    commit(i, &m);
}
void PID_Calc_Position(int i, float ticks)
{
    if (!isfinite(ticks) || (double)ticks >= 9223372036854775808.0 ||
        (double)ticks < -9223372036854775808.0) { D3508_Disable(i); return; }
    PID_Calc_PositionDt(i, (int64_t)ticks, D3508_DEFAULT_DT);
}
void PID_Calc_Torque(int i, float nm)
{
    if (!valid(i)) return;
    if (!isfinite(nm)) { D3508_Disable(i); return; }
    Motor_3508_T m;
    if (!control_snapshot(i, &m)) return;
    float current = (nm / D3508_ROTOR_TORQUE_CONSTANT) * D3508_CURRENT_UNITS_PER_AMP;
    if (!isfinite(current)) { D3508_Disable(i); return; }
    m.torque_err_sum = 0; m.Out_Current = Power_Limit(current, m.filter_speed);
    m.target_valid = 1;
    commit(i, &m);
}
static uint8_t mit_calc(Motor_3508_T *m, int64_t ticks, float rpm, float kp, float kd, float ff, float limit)
{
    float torque = kp * tick_error(ticks, m->total_angle) * D3508_RAD_PER_TICK +
                   kd * (rpm - m->filter_speed) * D3508_RPM_TO_WHEEL_RADPS + ff;
    if (!isfinite(torque)) return 0;
    float current = (clip(torque, limit) / D3508_REDUCTION_RATIO /
                     D3508_ROTOR_TORQUE_CONSTANT) * D3508_CURRENT_UNITS_PER_AMP;
    if (!isfinite(current)) return 0;
    m->target_angle = ticks; m->target_speed = rpm; m->target_valid = 1;
    m->speed_err_sum = m->position_err_sum = m->torque_err_sum = m->err_last = 0;
    m->Out_Current = Power_Limit(current, m->filter_speed);
    return 1;
}
void MIT_Wheel_Control(int i, int64_t ticks, float rpm, float kp, float kd, float ff, float limit)
{
    if (!valid(i)) return;
    if (!isfinite(rpm) || !isfinite(kp) || !isfinite(kd) || !isfinite(ff) ||
        !isfinite(limit) || kp < 0 || kd < 0 || limit < 0) { D3508_Disable(i); return; }
    Motor_3508_T m;
    if (!control_snapshot(i, &m)) return;
    if (!mit_calc(&m, ticks, rpm, kp, kd, ff, limit)) { D3508_Disable(i); return; }
    commit(i, &m);
}
static Motor3508_Status state_status(D3508_State state)
{
    switch (state) {
    case D3508_ENABLED: return MOTOR3508_OK;
    case D3508_DISABLED: return MOTOR3508_DISABLED;
    case D3508_FEEDBACK_TIMEOUT: return MOTOR3508_FEEDBACK_TIMEOUT;
    case D3508_BUS_FAULT: return MOTOR3508_BUS_FAULT;
    default: return MOTOR3508_NOT_READY;
    }
}
static void set_status(int b, Motor3508_Status status)
{
    uint32_t key = Driver_Lock();
    motor3508.last_set_status = status;
    if (b == 0) motor3508.Set_status_port1 = status;
    if (b == 1) motor3508.Set_status_port2 = status;
    Driver_Unlock(key);
}
static int command_index(int b, uint8_t id)
{
    return b >= 0 && id >= (b ? 3 : 1) && id <= (b ? 4 : 2) ? (int)id - 1 : -1;
}
void Motor3508_SetCommand(FDCAN_HandleTypeDef *h, uint8_t id, int64_t position,
                        float rpm, float torque, float kp, float kd)
{
    int b = bus_index(h), i = command_index(b, id);
    if (b < 0) { set_status(b, MOTOR3508_INVALID_ARGUMENT); return; }
    if (i < 0) { set_status(b, MOTOR3508_INVALID_ID); return; }
    if (!isfinite(rpm) || !isfinite(torque) || !isfinite(kp) || !isfinite(kd) || kp < 0 || kd < 0) {
        D3508_Disable(i); set_status(b, MOTOR3508_INVALID_ARGUMENT); return;
    }
    Motor_3508_T m; D3508_Feedback f;
    D3508_GetFeedback(i, &f);
    if (f.motor.state != D3508_ENABLED) { set_status(b, state_status(f.motor.state)); return; }
    m = f.motor;
    rpm = clip(rpm, D3508_MIT_MAX_ROTOR_RPM);
    torque = clip(torque, D3508_MIT_MAX_WHEEL_TORQUE_NM);
    if (!mit_calc(&m, position, rpm, kp, kd, torque, D3508_MIT_MAX_WHEEL_TORQUE_NM)) {
        D3508_Disable(i); set_status(b, MOTOR3508_INVALID_ARGUMENT); return;
    }
    commit(i, &m);
    uint32_t key = Driver_Lock();
    Motor3508_Status status = state_status(Motors[i].state);
    if (status == MOTOR3508_OK && Motors[i].epoch == m.epoch) {
        Motor3508_Command *cmd = &motor3508.cmd[id];
        cmd->position = position; cmd->velocity = rpm; cmd->torque = torque; cmd->kp = kp; cmd->kd = kd;
    } else if (status == MOTOR3508_OK) status = MOTOR3508_NOT_READY;
    Driver_Unlock(key);
    set_status(b, status);
}
Motor3508_Status Motor3508_GetCommandSnapshot(FDCAN_HandleTypeDef *h, uint8_t id, Motor3508_Command *out)
{
    int b = bus_index(h);
    if (b < 0 || !out) return MOTOR3508_INVALID_ARGUMENT;
    if (command_index(b, id) < 0) return MOTOR3508_INVALID_ID;
    uint32_t key = Driver_Lock(); *out = motor3508.cmd[id]; Driver_Unlock(key);
    return MOTOR3508_OK;
}
static HAL_StatusTypeDef send_bus(unsigned b)
{
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier = 0x200; tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME; tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE; tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN; tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    uint8_t data[8] = {0};
    for (int i = (int)b * 2; i < (int)b * 2 + 2; ++i) {
        D3508_Feedback f; D3508_GetFeedback(i, &f);
        int16_t c = f.motor.state == D3508_ENABLED && f.motor.target_valid ? f.motor.Out_Current : 0;
        if (c > D3508_CURRENT_LIMIT_UNITS) c = D3508_CURRENT_LIMIT_UNITS;
        if (c < -D3508_CURRENT_LIMIT_UNITS) c = -D3508_CURRENT_LIMIT_UNITS;
        data[i * 2] = (uint16_t)c >> 8; data[i * 2 + 1] = (uint8_t)c;
    }
    uint32_t key = Driver_Lock();
    uint8_t fault = buses[b].fault_latched, flush = flush_pending[b];
    flush_pending[b] = 0; Driver_Unlock(key);
    if (flush) (void)HAL_FDCAN_AbortTxRequest(handle(b), FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1);
    uint8_t full = !fault && HAL_FDCAN_GetTxFifoFreeLevel(handle(b)) == 0;
    HAL_StatusTypeDef status = fault ? HAL_ERROR : (full ? HAL_BUSY : HAL_FDCAN_AddMessageToTxFifoQ(handle(b), &tx, data));
    Motor3508_Status result = fault ? MOTOR3508_BUS_FAULT :
        (status == HAL_OK ? MOTOR3508_OK : (status == HAL_BUSY ? MOTOR3508_BUSY : MOTOR3508_CAN_ERROR));
    key = Driver_Lock(); buses[b].last_tx_status = status;
    if (status != HAL_OK) ++buses[b].tx_errors;
    if (full) ++buses[b].fifo_full_count;
    motor3508.last_send_status = result;
    if (b == 0) motor3508.Send_status_port1 = result;
    else motor3508.Send_status_port2 = result;
    Driver_Unlock(key);
    return status;
}
void Motor3508_SendCommand(FDCAN_HandleTypeDef *h)
{
    int b = bus_index(h);
    if (b < 0) {
        uint32_t key = Driver_Lock(); motor3508.last_send_status = MOTOR3508_INVALID_ARGUMENT; Driver_Unlock(key);
        return;
    }
    D3508_Maintain();
    (void)send_bus((unsigned)b);
}
D3508_SendResult D3508_SendCurrent(void)
{
    D3508_Maintain();
    D3508_SendResult result;
    result.bus1 = send_bus(0); result.bus2 = send_bus(1);
    return result;
}
void send_current(void) { (void)D3508_SendCurrent(); }
