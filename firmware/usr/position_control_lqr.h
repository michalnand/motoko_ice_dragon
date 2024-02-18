#ifndef _POSITION_CONTROL_LQR_H_
#define _POSITION_CONTROL_LQR_H_

#include <shaper.h>
#include <lqr.h>

class PositionControlLQR
{   
    public:
        void init();
        
        //x      required position
        //theta  required theta
        void step(float x, float theta);

        void callback();

    private:
        Shaper shaper_left, shaper_right;  
        LQR<2, 2> lqr;

    private:
        float wheel_diameter;
        float wheel_brace;

        float x, theta;

        uint32_t steps;
};

#endif