#ifndef _POSITION_CONTROL_LQR_H_
#define _POSITION_CONTROL_LQR_H_

#include <shaper.h>
#include <lqr.h>

class PositionControlLQR
{   
    public:
        void init();
        
        //required position and angle
        void set(float req_distance, float req_angle);

        void callback();

    public:
        Shaper distance_shaper, angle_shaper;
        LQR<2, 2> lqr; 

    private: 
        float wheel_diameter;
        float wheel_brace;   

        float shaper_ramp;  
        float speed_max;

        float req_distance, req_angle;

    public:
        float distance; 
        float angle;
        float distance_velocity;
        float angle_velocity;


        uint32_t steps;
};

#endif