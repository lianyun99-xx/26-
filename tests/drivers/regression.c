#include "3508_driver.h"
#include "Attitude_solution.h"
#include "cmsis_os2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
static unsigned assertions;
/* The bundled LCC runtime declares fmaxf but does not implement it. */
float fmaxf(float a, float b) { if (a != a) return b; if (b != b) return a; return a > b ? a : b; }
#define CHECK(x) do { ++assertions; if (!(x)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #x); exit(1); } } while (0)
#define NEAR(a,b,e) CHECK(fabs((double)(a)-(double)(b)) < (e))
uint32_t test_primask, SystemCoreClock = 400000000U;
static uint32_t tick, cycles, cycle_step = 100;
static osKernelState_t kernel = osKernelRunning;
static osThreadId_t thread = (void *)1;
static uint32_t read_cycles(void) { cycles += cycle_step; return cycles; }
TestCore test_core;
TestDWT test_dwt = {0, 0, read_cycles};
FDCAN_HandleTypeDef hfdcan1 = {FDCAN1}, hfdcan2 = {FDCAN2};
SPI_HandleTypeDef hspi2;
static unsigned fifo[2] = {2,2}, busoff[2], submits[2], aborts[2];
static HAL_StatusTypeDef tx_status[2], stop_status;
static uint8_t last_tx[2][8], regs[2][128];
static int cs[2] = {1,1}, spi_calls, fail_call;
static HAL_StatusTypeDef spi_failure;
static int bidx(const FDCAN_HandleTypeDef *h) { return h->Instance == FDCAN1 ? 0 : 1; }
static void outside_lock(void) { CHECK(test_primask == 0); }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { outside_lock(); tick += ms; }
osKernelState_t osKernelGetState(void) { return kernel; }
osThreadId_t osThreadGetId(void) { return thread; }
uint32_t osKernelGetTickFreq(void) { return 1000; }
int osDelay(uint32_t ms) { outside_lock(); tick += ms; return 0; }
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b,uint32_t c,uint32_t d)
{ outside_lock(); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h, const FDCAN_FilterTypeDef *f)
{ outside_lock(); CHECK(f->FilterID1 == (bidx(h) ? 0x203U : 0x201U)); CHECK(f->FilterID2 == f->FilterID1+1); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *h) { outside_lock(); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *h) { outside_lock(); return stop_status; }
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *h) { outside_lock(); busoff[bidx(h)] = 0; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h,uint32_t a,uint32_t b) { outside_lock(); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *h,uint32_t mask) { outside_lock(); ++aborts[bidx(h)]; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *h,FDCAN_ProtocolStatusTypeDef *p)
{ outside_lock(); p->BusOff = busoff[bidx(h)]; return HAL_OK; }
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *h) { outside_lock(); return fifo[bidx(h)]; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h,const FDCAN_TxHeaderTypeDef *t,const uint8_t *data)
{ outside_lock(); CHECK(t->Identifier == 0x200 && t->DataLength == FDCAN_DLC_BYTES_8); ++submits[bidx(h)]; memcpy(last_tx[bidx(h)],data,8); return tx_status[bidx(h)]; }
void HAL_GPIO_WritePin(void *port,uint16_t pin,GPIO_PinState state)
{ outside_lock(); int b = port == ACC_CS_GPIO_Port ? 0 : 1; cs[b] = state; if (!state) CHECK(cs[1-b] == 1); }
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,uint8_t *tx,uint8_t *rx,uint16_t n,uint32_t timeout)
{
    outside_lock(); CHECK(timeout == 1); CHECK(cs[0] != cs[1]); ++spi_calls;
    if (spi_calls == fail_call) return spi_failure;
    int b = cs[0] ? 1 : 0; unsigned reg = tx[0] & 0x7f;
    memset(rx,0,n);
    if (tx[0] & 0x80) {
        unsigned prefix = b ? 1 : 2;
        if (!b) CHECK(tx[1] == 0); /* Exactly one accelerometer dummy. */
        for (unsigned j = prefix; j < n; ++j) rx[j] = regs[b][reg+j-prefix];
    } else { CHECK(n == 2); regs[b][reg] = tx[1]; }
    return HAL_OK;
}
static uint8_t feedback(FDCAN_HandleTypeDef *h,unsigned id,unsigned angle,int speed)
{
    FDCAN_RxHeaderTypeDef r = {0}; r.Identifier = id; r.DataLength = FDCAN_DLC_BYTES_8;
    uint8_t d[8] = {0};
    d[0]=(uint8_t)(angle>>8); d[1]=(uint8_t)angle;
    d[2]=(uint8_t)((uint16_t)speed>>8); d[3]=(uint8_t)speed; d[5]=50; d[6]=30;
    return D3508_Decode(h,&r,d);
}
static void startup_enable_test(void)
{
    tick=0; busoff[0]=busoff[1]=0; stop_status=HAL_OK;
    D3508_Init();
    for (int i=0;i<4;++i) {
        CHECK(Motors[i].state==D3508_NOT_READY && !D3508_Enable(i));
        CHECK(!Motors[i].target_valid && Motors[i].Out_Current==0);
    }
    Motor3508_SetCommand(&hfdcan1,1,100,0,1,0,0);
    CHECK(motor3508.Set_status_port1==MOTOR3508_NOT_READY && !Motors[0].target_valid);
    CHECK(!feedback(&hfdcan2,0x201,100,0));
    CHECK(!feedback(&hfdcan1,0x201,8192,0));
    CHECK(Motors[0].state==D3508_NOT_READY);
    for (int i=0;i<4;++i) {
        FDCAN_HandleTypeDef *h=i<2?&hfdcan1:&hfdcan2;
        CHECK(feedback(h,0x201+i,100,0));
        CHECK(Motors[i].state==D3508_ENABLED);
        CHECK(!Motors[i].target_valid && Motors[i].Out_Current==0);
        if (i<3) CHECK(Motors[i+1].state==D3508_NOT_READY);
    }
    Motor3508_SendCommand(&hfdcan1); Motor3508_SendCommand(&hfdcan2);
    for (int b=0;b<2;++b) for (int j=0;j<8;++j) CHECK(last_tx[b][j]==0);
    for (int i=0;i<4;++i) {
        Motor3508_SetCommand(i<2?&hfdcan1:&hfdcan2,(uint8_t)(i+1),100,0,.5f,0,0);
        CHECK((i<2?motor3508.Set_status_port1:motor3508.Set_status_port2)==MOTOR3508_OK);
        CHECK(Motors[i].target_valid && Motors[i].Out_Current>0);
    }
    D3508_Disable(0);
    CHECK(feedback(&hfdcan1,0x201,110,0));
    CHECK(Motors[0].state==D3508_DISABLED && !Motors[0].target_valid);
    Motor3508_SetCommand(&hfdcan1,1,100,0,1,0,0);
    CHECK(motor3508.Set_status_port1==MOTOR3508_DISABLED && Motors[0].Out_Current==0);
    tick+=D3508_DEFAULT_TIMEOUT_MS;
    CHECK(feedback(&hfdcan1,0x202,200,0)); /* Expire inside Decode, without Maintain. */
    CHECK(Motors[1].state==D3508_DISABLED && !Motors[1].target_valid && Motors[1].Out_Current==0);
    CHECK(feedback(&hfdcan1,0x202,210,0)); CHECK(Motors[1].state==D3508_DISABLED);
    CHECK(D3508_Enable(1)); CHECK(!Motors[1].target_valid && Motors[1].Out_Current==0);
    Motor3508_SetCommand(&hfdcan1,2,210,0,.5f,0,0);
    CHECK(motor3508.Set_status_port1==MOTOR3508_OK && Motors[1].Out_Current>0);

    D3508_Init(); /* A fresh startup clears old commands and re-arms all four IDs. */
    for (int i=0;i<4;++i) {
        CHECK(Motors[i].state==D3508_NOT_READY && !Motors[i].target_valid && Motors[i].Out_Current==0);
        CHECK(motor3508.cmd[i+1].torque==0);
    }
    D3508_Disable(2); /* Cancel an initial request before any feedback. */
    D3508_OnBusOff(&hfdcan1); /* Also cancels IDs 1/2 before their first frames. */
    CHECK(!feedback(&hfdcan1,0x201,100,0));
    CHECK(D3508_RequestBusRecovery(1)); D3508_Maintain();
    for (int i=0;i<4;++i) {
        CHECK(feedback(i<2?&hfdcan1:&hfdcan2,0x201+i,100,0));
        CHECK(Motors[i].state==(i==3?D3508_ENABLED:D3508_DISABLED));
        CHECK(!Motors[i].target_valid && Motors[i].Out_Current==0);
    }
    D3508_Init();
    for (int i=0;i<4;++i) {
        CHECK(feedback(i<2?&hfdcan1:&hfdcan2,0x201+i,100,0));
        CHECK(Motors[i].state==D3508_ENABLED && !Motors[i].target_valid && Motors[i].Out_Current==0);
    }
    puts("PASS startup auto-enable for IDs 1-4, zero target gating and explicit fault recovery");
}
static void motors_test(void)
{
    D3508_Init(); CHECK(!D3508_Enable(0));
    CHECK(feedback(&hfdcan1,0x201,8000,0)); CHECK(Motors[0].total_angle == 8000);
    CHECK(Motors[0].state == D3508_ENABLED);
    CHECK(feedback(&hfdcan1,0x201,100,0)); CHECK(Motors[0].total_angle == 8292);
    CHECK(feedback(&hfdcan1,0x201,8100,0)); CHECK(Motors[0].total_angle == 8100);
    Motors[0].total_round = 262143; Motors[0].cur_angle = 8100;
    CHECK(feedback(&hfdcan1,0x201,10,0)); CHECK(Motors[0].total_angle == 2147483658LL);
    Motors[0].total_round = -262144; Motors[0].cur_angle = 10;
    CHECK(feedback(&hfdcan1,0x201,8100,0)); CHECK(Motors[0].total_angle == -2147483740LL);
    D3508_Init(); CHECK(!Motors[0].init_flag); CHECK(Motors[0].cur_angle == 0);
    CHECK(!feedback(&hfdcan2,0x201,100,0)); CHECK(!feedback(&hfdcan1,0x203,100,0));
    CHECK(!feedback(&hfdcan1,0x201,8192,0));
    FDCAN_RxHeaderTypeDef r = {0}; r.Identifier=0x201; r.DataLength=7;
    uint8_t d[8]={0}; CHECK(!D3508_Decode(&hfdcan1,&r,d));
    r.DataLength=FDCAN_DLC_BYTES_8; r.IdType=1; CHECK(!D3508_Decode(&hfdcan1,&r,d));
    r.IdType=0; r.RxFrameType=1; CHECK(!D3508_Decode(&hfdcan1,&r,d));
    r.RxFrameType=0; r.FDFormat=1; CHECK(!D3508_Decode(&hfdcan1,&r,d));
    CHECK(feedback(&hfdcan1,0x201,100,0));
    D3508_Disable(0);
    PID_Calc_Torque(0,0.1f); CHECK(Motors[0].Out_Current == 0);
    CHECK(D3508_Enable(0));
    PID_Calc_Speed(0); CHECK(!Motors[0].target_valid && Motors[0].Out_Current==0);
    Speed_PID.Kp=0.0f; Speed_PID.Ki=0.0f; Speed_PID.Kd=1.0f; Speed_PID.Max_Out=16000.0f; Speed_PID.Max_Sum=0.0f;
    CHECK(D3508_SetSpeedTarget(0,10)); PID_Calc_Speed(0);
    NEAR(Motors[0].Out_Current,2000,2);
    PID_Calc_Speed(0); CHECK(Motors[0].Out_Current==0);
    Clear_Motor_PID(0); CHECK(D3508_SetSpeedTarget(0,10)); PID_Calc_SpeedDt(0,.01f);
    NEAR(Motors[0].Out_Current,1000,2);
    Clear_Motor_PID(0); Speed_PID.Kd=0; Speed_PID.Ki=1; Speed_PID.Max_Sum=10000;
    D3508_SetSpeedTarget(0,10); PID_Calc_SpeedDt(0,.01f); NEAR(Motors[0].speed_err_sum,20,0.001);
    Clear_Motor_PID(0); Speed_PID.Ki=0; Speed_PID.Kd=1;
    Position_PID.Kp=0.0f; Position_PID.Ki=1.0f; Position_PID.Kd=0.0f; Position_PID.Max_Out=6000.0f; Position_PID.Max_Sum=10.0f;
    PID_Calc_PositionDt(0,200,0.005f); CHECK(Motors[0].position_err_sum==10);
    NEAR(Motors[0].err_last,0.05f,0.000001); /* No position overwrite of speed history. */
    Position_PID.Max_Sum=0; PID_Calc_PositionDt(0,200,0.005f); CHECK(Motors[0].position_err_sum==0);
    Motors[0].total_angle=2147483648LL;
    MIT_Wheel_Control(0,2147483649LL,0,100,0,0,10); CHECK(Motors[0].Out_Current>0);
    tick += 20; D3508_Maintain(); CHECK(Motors[0].state==D3508_FEEDBACK_TIMEOUT);
    CHECK(Motors[0].Out_Current==0 && Motors[0].target_angle==0 && Motors[0].err_last==0);
    CHECK(!D3508_Enable(0)); CHECK(feedback(&hfdcan1,0x201,4000,0));
    CHECK(Motors[0].state==D3508_DISABLED && Motors[0].total_angle==4000);
    PID_Calc_Torque(0,1); CHECK(Motors[0].Out_Current==0);
    CHECK(D3508_Enable(0)); CHECK(Motors[0].target_speed==0); PID_Calc_Torque(0,0.1f);
    CHECK(Motors[0].Out_Current>0);
    CHECK(feedback(&hfdcan2,0x203,20,0)); CHECK(D3508_Enable(2)); PID_Calc_Torque(2,-0.1f);
    D3508_SendResult sent=D3508_SendCurrent(); CHECK(sent.bus1==HAL_OK && sent.bus2==HAL_OK);
    CHECK(last_tx[0][0] || last_tx[0][1]); CHECK(last_tx[0][4]==0);
    CHECK(last_tx[1][0]==0 && last_tx[1][4]!=0);
    unsigned before=submits[0]; fifo[0]=0; sent=D3508_SendCurrent();
    CHECK(sent.bus1==HAL_BUSY && submits[0]==before); fifo[0]=2;
    tx_status[0]=HAL_ERROR; sent=D3508_SendCurrent(); CHECK(sent.bus1==HAL_ERROR); tx_status[0]=HAL_OK;
    busoff[0]=1; D3508_Maintain(); CHECK(Motors[0].state==D3508_BUS_FAULT);
    CHECK(Motors[2].state==D3508_ENABLED && !D3508_Enable(0));
    CHECK(D3508_RequestBusRecovery(1)); stop_status=HAL_ERROR; D3508_Maintain(); CHECK(!D3508_Enable(0));
    stop_status=HAL_OK; CHECK(D3508_RequestBusRecovery(1)); D3508_Maintain();
    CHECK(Motors[0].state==D3508_NOT_READY); CHECK(feedback(&hfdcan1,0x201,20,0));
    CHECK(Motors[0].state==D3508_DISABLED && Motors[0].Out_Current==0);
    D3508_BusStatus bs; CHECK(D3508_GetBusStatus(1,&bs)); CHECK(bs.tx_errors>=2 && bs.fifo_full_count==1 && bs.bus_off_count==1);
    CHECK(!D3508_GetFeedback(-1,NULL)); Clear_Motor_PID(4); PID_Calc_Torque(-1,0); PID_Calc_SpeedDt(4,0);
    CHECK(D3508_Enable(0)); PID_Calc_Torque(0,NAN); CHECK(Motors[0].state==D3508_DISABLED);
    CHECK(D3508_Enable(0)); PID_Calc_SpeedDt(0,INFINITY); CHECK(Motors[0].Out_Current==0);
    CHECK(D3508_Enable(0)); PID_Calc_Position(0,INFINITY); CHECK(Motors[0].state==D3508_DISABLED);
    CHECK(D3508_Enable(0)); MIT_Wheel_Control(0,0,0,NAN,0,0,10); CHECK(Motors[0].state==D3508_DISABLED);
    CHECK(!D3508_SetSpeedTarget(4,0)); CHECK(!D3508_GetBusStatus(3,&bs));
    CHECK(Power_Limit(NAN,0)==0); CHECK(Power_Limit(1e8f,0)==16000);
    tick=UINT32_MAX-5; D3508_Init(); feedback(&hfdcan1,0x201,5,0); tick=15; D3508_Maintain();
    CHECK(Motors[0].state==D3508_FEEDBACK_TIMEOUT);
    puts("PASS motor wrap/PID/routing/fault tests");
}
static void set_le(unsigned b,unsigned r,int value) { regs[b][r]=(uint8_t)value; regs[b][r+1]=(uint8_t)((uint16_t)value>>8); }
static void set_send_test(void)
{
    tick=0; busoff[0]=busoff[1]=0; D3508_Init();
    Motor3508_SetCommand(&hfdcan1,1,100,0,1,0,0);
    CHECK(motor3508.Set_status_port1==MOTOR3508_NOT_READY);
    for (unsigned i=0;i<4;++i) {
        CHECK(feedback(i<2?&hfdcan1:&hfdcan2,0x201+i,100,0)); CHECK(Motors[i].state==D3508_ENABLED);
    }
    unsigned s0=submits[0], s1=submits[1];
    Motor3508_SetCommand(&hfdcan1,1,100,0,1,0,0);
    CHECK(motor3508.Set_status_port1==MOTOR3508_OK); NEAR(Motors[0].Out_Current,3344.77,2);
    Motor3508_SetCommand(&hfdcan1,2,100,0,-1,0,0);
    CHECK(motor3508.Set_status_port1==MOTOR3508_OK); NEAR(Motors[1].Out_Current,-3344.77,2);
    CHECK(submits[0]==s0 && submits[1]==s1); /* Set never calls CAN transmit. */
    Motor3508_Command cmd;
    CHECK(Motor3508_GetCommandSnapshot(&hfdcan1,1,&cmd)==MOTOR3508_OK);
    CHECK(cmd.position==100 && cmd.torque==1 && cmd.kp==0);
    Motor3508_SendCommand(&hfdcan1);
    CHECK(motor3508.Send_status_port1==MOTOR3508_OK && submits[0]==s0+1 && submits[1]==s1);
    CHECK((int16_t)(((uint16_t)last_tx[0][0]<<8)|last_tx[0][1])==Motors[0].Out_Current);
    CHECK((int16_t)(((uint16_t)last_tx[0][2]<<8)|last_tx[0][3])==Motors[1].Out_Current);
    Motor3508_SetCommand(&hfdcan2,3,100,0,.5f,0,0);
    Motor3508_SetCommand(&hfdcan2,4,100,0,-.5f,0,0);
    Motor3508_SendCommand(&hfdcan2); CHECK(submits[1]==s1+1 && motor3508.Send_status_port2==MOTOR3508_OK);
    CHECK(last_tx[1][0]==0 && last_tx[1][2]==0 && last_tx[1][4]!=0 && last_tx[1][6]!=0);
    Motor3508_SetCommand(&hfdcan2,1,0,0,0,0,0); CHECK(motor3508.Set_status_port2==MOTOR3508_INVALID_ID);
    CHECK(Motor3508_GetCommandSnapshot(&hfdcan2,1,&cmd)==MOTOR3508_INVALID_ID);
    Motor3508_SetCommand(&hfdcan1,1,100,20000,100,0,0);
    CHECK(motor3508.Set_status_port1==MOTOR3508_OK);
    Motor3508_GetCommandSnapshot(&hfdcan1,1,&cmd);
    CHECK(cmd.velocity==D3508_MIT_MAX_ROTOR_RPM && cmd.torque==D3508_MIT_MAX_WHEEL_TORQUE_NM);
    CHECK(abs(Motors[0].Out_Current)<=D3508_CURRENT_LIMIT_UNITS);
    Motor3508_SetCommand(&hfdcan1,1,100,0,0,100,0); /* MIT P is calculated in Set. */
    CHECK(Motors[0].Out_Current==0);
    CHECK(feedback(&hfdcan1,0x201,90,0));
    Motor3508_SendCommand(&hfdcan1); CHECK(last_tx[0][0]==0 && last_tx[0][1]==0); /* No implicit recalc. */
    Motor3508_SetCommand(&hfdcan1,1,100,0,0,100,0); CHECK(Motors[0].Out_Current>0);
    fifo[0]=0; s0=submits[0]; Motor3508_SendCommand(&hfdcan1);
    CHECK(motor3508.Send_status_port1==MOTOR3508_BUSY && submits[0]==s0); fifo[0]=2;
    Motor3508_SetCommand(&hfdcan1,1,0,NAN,0,0,0);
    CHECK(motor3508.Set_status_port1==MOTOR3508_INVALID_ARGUMENT && Motors[0].Out_Current==0);
    CHECK(Motors[0].state==D3508_DISABLED);
    Motor3508_SetCommand(NULL,1,0,0,0,0,0); CHECK(motor3508.last_set_status==MOTOR3508_INVALID_ARGUMENT);
    Motor3508_SendCommand(NULL); CHECK(motor3508.last_send_status==MOTOR3508_INVALID_ARGUMENT);
    tick+=20; Motor3508_SetCommand(&hfdcan2,3,0,0,0,0,0);
    CHECK(motor3508.Set_status_port2==MOTOR3508_FEEDBACK_TIMEOUT);
    Motor3508_SendCommand(&hfdcan2); CHECK(last_tx[1][4]==0 && last_tx[1][5]==0);
    busoff[0]=1; Motor3508_SendCommand(&hfdcan1); CHECK(motor3508.Send_status_port1==MOTOR3508_BUS_FAULT);
    busoff[0]=0;
    puts("PASS MIT Set calculation, macro limits, grouped Send and per-port statuses");
}
static void bmi_test(void)
{
    tick=0; regs[0][0]=BMI088_ACC_CHIP_ID_VALUE; regs[1][0]=BMI088_GYRO_CHIP_ID_VALUE;
    CHECK(BMI088_init()==0); CHECK(cs[0] && cs[1]);
    CHECK(regs[0][BMI088_INT_MAP_DATA]==0 && regs[1][BMI088_GYRO_CTRL]==0 && regs[1][BMI088_GYRO_INT3_INT4_IO_MAP]==0);
    set_le(0,0x12,-100); set_le(0,0x14,200); set_le(0,0x16,10919);
    set_le(1,2,-300); set_le(1,4,400); set_le(1,6,-500);
    regs[0][0x22]=0xff; regs[0][0x23]=0xe0; /* Signed 11-bit -1. */
    BMI088_Sample s, previous; CHECK(BMI088_ReadSample(&s)==BMI088_STATUS_OK);
    NEAR(s.accel[0],-100*BMI088_ACCEL_SEN,0.00001); NEAR(s.gyro[2],-500*BMI088_GYRO_SEN,0.00001); NEAR(s.temperature,22.875,0.00001);
    CHECK(s.sequence==1); previous=s;
    for (int part=1;part<=3;++part) {
        fail_call=spi_calls+part; spi_failure=part==2?HAL_BUSY:HAL_TIMEOUT; tick+=5;
        CHECK(BMI088_ReadSample(&s)!=BMI088_STATUS_OK); CHECK(cs[0] && cs[1]);
        CHECK(s.sequence==previous.sequence && s.timestamp_ms==previous.timestamp_ms);
        BMI088_GetSample(&s); CHECK(s.timestamp_ms==previous.timestamp_ms);
        BMI088_Diagnostics diag; BMI088_GetDiagnostics(&diag); CHECK(diag.last_spi_status==spi_failure);
        fail_call=0;
    }
    float old_gyro[3]={1.0f,2.0f,3.0f}, old_accel[3]={4.0f,5.0f,6.0f}, old_temp=7.0f;
    fail_call=spi_calls+1; spi_failure=HAL_BUSY;
    CHECK(!BMI088_read(old_gyro,old_accel,&old_temp)); CHECK(old_gyro[0]==1 && old_accel[2]==6 && old_temp==7); fail_call=0;
    thread=(void *)2; CHECK(BMI088_ReadSample(&s)==BMI088_STATUS_BUSY); thread=(void *)1;
    CHECK(BMI088_CycleElapsed(UINT32_MAX-10,20,30)); CHECK(!BMI088_CycleElapsed(UINT32_MAX-10,18,30));
    cycles=UINT32_MAX-100; CHECK(BMI088_delay_us(10)); cycle_step=0; CHECK(!BMI088_delay_us(1)); cycle_step=100;
    kernel=osKernelInactive; uint32_t t=tick; BMI088_delay_ms(3); CHECK(tick==t+3); kernel=osKernelRunning;
    set_le(0,0x12,0); set_le(0,0x14,0); set_le(0,0x16,10919); set_le(1,2,0); set_le(1,4,0); set_le(1,6,0);
    Attitude_Init(); BMI088_ReadSample(&s); Attitude_Update(.005f); CHECK(!g_Attitude.Data_Valid);
    CHECK(BMI088_RequestCalibration()); CHECK(!BMI088_RequestCalibration());
    BMI088_ReadSample(&s); BMI088_CalibrationStep(&s);
    BMI088_Calibration c; BMI088_GetCalibration(&c); CHECK(c.consecutive_samples==1);
    for (int k=0;k<10;++k) BMI088_CalibrationStep(&s);
    BMI088_GetCalibration(&c); CHECK(c.consecutive_samples==1);
    tick+=5; BMI088_ReadSample(&s); s.gyro[0]=1; BMI088_CalibrationStep(&s);
    BMI088_GetCalibration(&c); CHECK(c.consecutive_samples==0);
    for (unsigned k=0;k<400;++k) { tick+=5; CHECK(BMI088_ReadSample(&s)==BMI088_STATUS_OK); BMI088_CalibrationStep(&s); }
    BMI088_GetCalibration(&c); CHECK(c.state==BMI088_CAL_SUCCEEDED && c.consecutive_samples==400);
    Attitude_Update(.005f); CHECK(g_Attitude.Data_Valid && g_Attitude.Calibrated);
    uint32_t valid_time=g_Attitude.Last_Valid_Update_ms;
    float yaw=g_Attitude.Angle_Yaw; tick+=5; Attitude_Update(.005f);
    CHECK(g_Attitude.Last_Valid_Update_ms==valid_time && g_Attitude.Angle_Yaw==yaw);
    fail_call=spi_calls+1; spi_failure=HAL_TIMEOUT; CHECK(BMI088_ReadSample(&s)!=BMI088_STATUS_OK);
    Attitude_Update(.005f); CHECK(!g_Attitude.Data_Valid && g_Attitude.Last_Valid_Update_ms==valid_time); fail_call=0;
    tick+=5; BMI088_ReadSample(&s); Attitude_Update(.005f); CHECK(g_Attitude.Data_Valid);
    valid_time=g_Attitude.Last_Valid_Update_ms; tick+=50; Attitude_Update(.005f);
    CHECK(!g_Attitude.Data_Valid && g_Attitude.Last_Valid_Update_ms==valid_time);
    Attitude_Data_t snapshot; CHECK(Attitude_GetSnapshot(&snapshot)); CHECK(!snapshot.Data_Valid);
    tick+=5; BMI088_ReadSample(&s); s.accel[2]=-9.8f; Attitude_ProcessSample(&s,.005f); CHECK(!g_Attitude.Data_Valid);
    s.accel[2]=NAN; Attitude_ProcessSample(&s,.005f); CHECK(!g_Attitude.Data_Valid);
    CHECK(BMI088_RequestCalibration()); BMI088_CalibrationStep(NULL); tick+=2500; BMI088_CalibrationStep(NULL);
    BMI088_GetCalibration(&c); CHECK(c.state==BMI088_CAL_FAILED); Attitude_Update(.005f); CHECK(!g_Attitude.Calibrated);
    regs[0][0]=0; regs[1][0]=0; CHECK(BMI088_init()!=0);
    BMI088_Diagnostics diag; BMI088_GetDiagnostics(&diag); CHECK(diag.accel_init_error==BMI088_NO_SENSOR && diag.gyro_init_error==BMI088_NO_SENSOR);
    puts("PASS BMI transactions/calibration/attitude tests");
}
static void startup_read_test(void)
{
    kernel=osKernelInactive; thread=NULL;
    regs[0][0]=BMI088_ACC_CHIP_ID_VALUE; regs[1][0]=BMI088_GYRO_CHIP_ID_VALUE;
    set_le(0,0x12,0); set_le(0,0x14,0); set_le(0,0x16,10919);
    set_le(1,2,0); set_le(1,4,0); set_le(1,6,0);
    CHECK(BMI088_init()==0); Attitude_Init(); /* Exactly the main startup order. */
    BMI088_Sample s; CHECK(!BMI088_GetSample(&s));
    kernel=osKernelRunning; thread=(void *)1;
    int calls=spi_calls;
    CHECK(!BMI088Read()); CHECK(spi_calls==calls+3); /* Owner migration, no reset/init. */
    CHECK(BMI088_GetSample(&s) && s.sequence==1);
    CHECK(BMI088_RequestCalibration());
    for (unsigned k=0;k<400;++k) {
        tick+=5; calls=spi_calls;
        uint8_t ready=BMI088Read();
        CHECK(spi_calls==calls+3); CHECK(ready==(k==399));
    }
    Attitude_Data_t a; CHECK(Attitude_GetSnapshot(&a) && a.Data_Valid);
    CHECK(a.EKF_Valid);
    NEAR(a.Quaternion_W*a.Quaternion_W + a.Quaternion_X*a.Quaternion_X +
         a.Quaternion_Y*a.Quaternion_Y + a.Quaternion_Z*a.Quaternion_Z, 1.0, 0.0001);
    NEAR(a.Angle_Pitch,0,0.001); NEAR(a.Angle_Roll,0,0.001);
    {
        BMI088_Sample motion = s;
        float pitch_before = a.Angle_Pitch;
        tick += 5;
        motion.sequence += 1;
        motion.timestamp_ms = tick;
        motion.accel[2] = 2.0f * BMI088_GRAVITY; /* Skip accel correction. */
        motion.gyro[0] = 1.0f;
        motion.gyro[1] = 0.0f;
        motion.gyro[2] = 0.0f;
        Attitude_ProcessSample(&motion, 0.005f);
        CHECK(g_Attitude.Data_Valid && g_Attitude.EKF_Valid);
        CHECK(g_Attitude.Angle_Pitch > pitch_before);
        {
            float pitch_after_gyro = g_Attitude.Angle_Pitch;
            unsigned sample_index;
            for (sample_index = 0; sample_index < 6; ++sample_index) {
                tick += 5;
                motion.sequence += 1;
                motion.timestamp_ms = tick;
                motion.accel[2] = BMI088_GRAVITY;
                motion.gyro[0] = 0.0f;
                Attitude_ProcessSample(&motion, 0.005f);
            }
            CHECK(g_Attitude.Data_Valid && g_Attitude.EKF_Valid);
            CHECK(fabs(g_Attitude.Angle_Pitch) < fabs(pitch_after_gyro));
        }
        NEAR(g_Attitude.Quaternion_W*g_Attitude.Quaternion_W +
             g_Attitude.Quaternion_X*g_Attitude.Quaternion_X +
             g_Attitude.Quaternion_Y*g_Attitude.Quaternion_Y +
             g_Attitude.Quaternion_Z*g_Attitude.Quaternion_Z, 1.0, 0.0001);
        Attitude_ResetYaw();
        tick += 5;
        motion.sequence += 1;
        motion.timestamp_ms = tick;
        motion.accel[2] = BMI088_GRAVITY;
        motion.gyro[0] = 0.0f;
        Attitude_ProcessSample(&motion, 0.005f);
        CHECK(g_Attitude.Data_Valid && fabs(g_Attitude.Angle_Yaw) < 0.01);
    }
    CHECK(Attitude_GetSnapshot(&a));
    uint32_t valid_time=a.Last_Valid_Update_ms;
    tick+=5; fail_call=spi_calls+1; spi_failure=HAL_TIMEOUT;
    CHECK(!BMI088Read()); CHECK(Attitude_GetSnapshot(&a) && !a.Data_Valid);
    CHECK(a.Last_Valid_Update_ms==valid_time); fail_call=0;
    tick+=5; CHECK(BMI088Read()); CHECK(Attitude_GetSnapshot(&a) && a.Data_Valid);
    calls=spi_calls; thread=(void *)2;
    CHECK(!BMI088Read()); CHECK(spi_calls==calls); /* Other task cannot acquire SPI. */
    thread=(void *)1; CHECK(Attitude_GetSnapshot(&a) && a.Data_Valid);
    puts("PASS pre-RTOS init -> BMI088Read task ownership, complete pose pipeline, no repeated init");
}
int main(void)
{
    motors_test(); startup_enable_test(); set_send_test(); bmi_test(); startup_read_test(); CHECK(test_primask==0);
    printf("PASS: %u C assertions\n",assertions);
    return 0;
}
