#include "timer.h"
#include <device.h>


volatile uint32_t g_time = 0;

#ifdef __cplusplus
extern "C" {
#endif

void SysTick_Handler(void)
{
    g_time++;
}

#ifdef __cplusplus
}
#endif


Timer::Timer()
{
    
} 



void Timer::init(uint32_t frequency)
{
    g_time      = 0;

    //interrupt every 1ms 
    SysTick_Config(SystemCoreClock/frequency);
    __enable_irq();
}

void Timer::delay_ms(uint32_t time_ms)
{
    time_ms = time_ms + g_time;
    while (time_ms > g_time)
    {
        __asm("wfi");
    }
}