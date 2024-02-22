#ifndef _POSITION_CONTROL_LQR_H_
#define _POSITION_CONTROL_LQR_H_

#include <shaper_s.h>
#include <lqr.h> 

class PositionControlLQR
{   
    public:
        void init();
        
        //required position and angle
        void set(float req_distance, float req_angle);

        void callback();

    public:
        ShaperS distance_shaper, angle_shaper;
        LQR<4, 2> lqr; 

    private: 
        float wheel_diameter;
        float wheel_brace;   

        float shaper_ramp;  
        float speed_max;

        float req_distance, req_angle;

    public:
        float distance_prev; 
        float distance; 
        float angle_prev;
        float angle;
        

        uint32_t steps;
};

#endif