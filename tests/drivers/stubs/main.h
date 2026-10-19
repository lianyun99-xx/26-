#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#define __MAIN_H /* Suppress generated main.h when driver_sync includes it. */
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { void *Instance; } FDCAN_HandleTypeDef;
typedef struct { uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2; } FDCAN_FilterTypeDef;
typedef struct { uint32_t Identifier, IdType, RxFrameType, DataLength, FDFormat; } FDCAN_RxHeaderTypeDef;
typedef struct {
    uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator,
             BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t BusOff; } FDCAN_ProtocolStatusTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
#define FDCAN1 ((void *)1)
#define FDCAN2 ((void *)2)
#define FDCAN3 ((void *)3)
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_FILTER_DUAL 1U
#define FDCAN_FILTER_TO_RXFIFO0 1U
#define FDCAN_REJECT 2U
#define FDCAN_REJECT_REMOTE 1U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_BUS_OFF 2U
#define FDCAN_TX_BUFFER0 1U
#define FDCAN_TX_BUFFER1 2U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_NO_TX_EVENTS 0U
#define ACC_CS_GPIO_Port ((void *)1)
#define GYRO_CS_GPIO_Port ((void *)2)
#define ACC_CS_Pin 1
#define GYRO_CS_Pin 2
extern uint32_t test_primask, SystemCoreClock;
static inline uint32_t __get_PRIMASK(void) { return test_primask; }
static inline void __disable_irq(void) { test_primask = 1; }
static inline void __set_PRIMASK(uint32_t x) { test_primask = x; }
#define __DMB() ((void)0)
#define __DSB() ((void)0)
#define __ISB() ((void)0)
typedef struct { uint32_t DEMCR; } TestCore;
typedef struct { uint32_t CTRL, LAR; uint32_t (*read_cycles)(void); } TestDWT;
extern TestCore test_core;
extern TestDWT test_dwt;
#define CoreDebug (&test_core)
#define DWT (&test_dwt)
#define CYCCNT read_cycles()
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, const FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(const FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(const FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, const FDCAN_TxHeaderTypeDef *, const uint8_t *);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
void HAL_GPIO_WritePin(void *, uint16_t, GPIO_PinState);
#endif
