#include "lqr_single.h"


LQRSingle::LQRSingle()
{
 
}

LQRSingle::~LQRSingle()
{

} 

void LQRSingle::init(float k, float ki, float antiwindup)
{
    this->k = k;
    this->ki = ki;
 
    this->integral_action = 0.0;
    this->antiwindup = antiwindup;
}
 
        
//xr  required value 
//x   actual value
float LQRSingle::step(float xr, float x)
{
    //integral action
    //error = xr - x
    //integral_action+= ki@error * dt
    float error = xr - x;   
    float integral_action_new = this->integral_action + ki*error;

    //LQR controller with integral action
    //u = -k@x + ki@error_sum
    float u_new = -k*x + this->integral_action;

    //antiwindup with conditional integration
    float u = _clip(u_new, -antiwindup, antiwindup);

    this->integral_action = integral_action_new - (u_new - u);
    
    return u;
}

float LQRSingle::_clip(float v, float min_v, float max_v)
{
    if (v < min_v)
    {
        v = min_v;
    }
    else if (v > max_v)
    {
        v = max_v;
    }

    return v;
}
