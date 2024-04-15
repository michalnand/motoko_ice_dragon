#ifndef _CLOCK_H_
#define _CLOCK_H_

#include <stm32f7xx.h>

#define SysClok216_8HSE   ((uint32_t)0)
#define SysClok312_8HSE   ((uint32_t)1)
#define SysClok216_24HSE   ((uint32_t)2)
#define SysClok312_24HSE   ((uint32_t)3)
 

void SetSysClock(uint32_t mode = SysClok216_8HSE);


inline uint32_t timer_period(uint32_t dt_us)
{
    return ((SystemCoreClock/1000)*dt_us)/1000 + 1;
}

#endif
