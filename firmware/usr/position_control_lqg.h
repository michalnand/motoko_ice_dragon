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
        void stop();

        //set circular motion
        void set_circle_motion(float radius, float speed);

        //5mm, 5degrees
        bool on_target(float distance_threshold = 5.0, float angle_threshold = 0.0872);

        void enable_lf();
        void disable_lf();
        
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


extern PositionControlLQG position_control;

#endif