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

        void set_circle_motion(float radius, float speed);

        void enable_lf();
        void disable_lf();
        
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

        bool lf_mode;

    public:
        float distance_prev; 
        float distance; 
        float angle_prev;
        float angle;

        float line_angle_prev;
        float line_angle; 

        uint32_t steps;
};

#endif