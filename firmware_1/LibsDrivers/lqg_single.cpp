#include "lqg_single.h"


LQGSingle::LQGSingle()
{
 
}

LQGSingle::~LQGSingle()
{

} 

void LQGSingle::init(float a, float b, float k, float ki, float f, float antiwindup)
{
    this->a  = a;
    this->b  = b;
    this->k  = k;
    this->ki = ki;
    this->f  = f;

    this->x_hat = 0.0;

    this->integral_action = 0.0;

    this->antiwindup = antiwindup;
}
 
        
//xr  required value
//x   actual value
float LQGSingle::step(float xr, float x)
{
    //integral action
    //error = xr - x
    //integral_action+= ki@error * dt
    float error = xr - x;   
    float integral_action_new = this->integral_action + ki*error;


    //LQR controller with integral action
    //u = -k@x + ki@error_sum
    float u_new = -k*x_hat + this->integral_action;

    //antiwindup with conditional integration
    float u = _clip(u_new, -antiwindup, antiwindup);

    this->integral_action = integral_action_new - (u_new - u);
    
    // kalman observer, prediction and correction
    float e         = x - x_hat;
    x_hat           = a*x_hat + b*u + f*e;

    return u;
}

float LQGSingle::get_x_hat()
{
    return x_hat;
}

float LQGSingle::_clip(float v, float min_v, float max_v)
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
