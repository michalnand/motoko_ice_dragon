#ifndef _TIMER_H_
#define _TIMER_H_

#include <stdint.h>


class Timer
{
    public:
        Timer();

        void init(uint32_t frequency = 1000);

    public:
        uint32_t get_time();
        void delay_ms(uint32_t time_ms);

        void reset();
};

#endif