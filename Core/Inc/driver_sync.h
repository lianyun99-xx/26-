#ifndef DRIVER_SYNC_H
#define DRIVER_SYNC_H
#include "main.h"
/* Only copy snapshots / change integer state inside these sections. */
static inline uint32_t Driver_Lock(void)
{
    uint32_t key = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return key;
}
static inline void Driver_Unlock(uint32_t key)
{
    __DMB();
    __set_PRIMASK(key);
}
#endif
