#ifndef TEST_OS_H
#define TEST_OS_H
#include <stdint.h>
typedef void *osThreadId_t;
typedef enum { osKernelInactive, osKernelReady, osKernelRunning } osKernelState_t;
osKernelState_t osKernelGetState(void);
osThreadId_t osThreadGetId(void);
uint32_t osKernelGetTickFreq(void);
int osDelay(uint32_t ticks);
#endif
