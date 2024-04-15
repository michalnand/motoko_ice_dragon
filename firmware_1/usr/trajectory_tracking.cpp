#include "trajectory_tracking.h"

#include <position_control_lqg.h>

#include <fmath.h>




void TrajectoryTracking::start(float target_distance, float target_angle)
{
    this->start_distance  = position_control.distance;
    this->target_distance = this->start_distance + target_distance;
    this->target_distance_curr = this->start_distance;

    this->start_angle  = position_control.angle;
    this->target_angle = this->start_angle + target_angle;
}



bool TrajectoryTracking::step(float distance_th, float angle_th)
{
    float cur_distance = position_control.distance;
    float cur_angle    = position_control.angle; 

    float kf = 0.05; 
    this->target_distance_curr = (1.0 - kf)*this->target_distance_curr  + kf*this->target_distance;
    

    float k = 1.0/(this->target_distance_curr - this->start_distance);
    float q = 1.0 - k*this->target_distance_curr;
    float w = k*cur_distance + q;

    if (this->target_distance_curr < this->start_distance)
    {
        //have no idea why robot needs turn more sharp when going back
        w = 2.5*w; 
    } 

    w = clip(w, 0.0, 1.0);

    float req_angle = (1.0 - w)*this->start_angle + w*this->target_angle;


    position_control.set(this->target_distance_curr, req_angle);


    float error_d = abs(cur_distance - this->target_distance_curr);
    float error_a = abs(cur_angle - this->target_angle);

    if ((error_d < distance_th) && (error_a < angle_th))
    {
        return true;
    }

    return false;   
}
