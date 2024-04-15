#ifndef _TRAJECTORY_TRACKING_H_
#define _TRAJECTORY_TRACKING_H_

#include <fmath.h>



class TrajectoryTracking    
{
    public:
        void start(float target_distance, float target_angle);
        bool step(float distance_th = 4.0, float angle_th = 4.0*PI/180.0);

    private:
        float start_distance, start_angle;
        float target_distance, target_angle;
        float target_distance_curr;
};


#endif