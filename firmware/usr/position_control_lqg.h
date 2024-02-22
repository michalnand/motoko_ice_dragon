#ifndef _POSITION_CONTROL_LQG_H_
#define _POSITION_CONTROL_LQG_H_

#include <shaper_s.h>
#include <lqg.h>

class PositionControlLQG
{   
    public:
        void init();
        
        //required position and angle
        void set(float req_distance, float req_angle);

        void callback();

    public:
        ShaperS distance_shaper, angle_shaper;
        LQG<4, 2, 4> lqg;  

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