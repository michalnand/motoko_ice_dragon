#ifndef _POSITION_CONTROL_LQG_H_
#define _POSITION_CONTROL_LQG_H_

#include <shaper.h>
#include <lqg.h>

class PositionControlLQG
{   
    public:
        void init();
        
        //x      required position
        //theta  required theta
        void set(float x, float theta);

        void callback();

    public:
        Shaper shaper_left, shaper_right;  
        LQG<4, 2, 2> lqg;

    private:
        float wheel_diameter;
        float wheel_brace;

        float x, theta;

        uint32_t steps;
};

#endif