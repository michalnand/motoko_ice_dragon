#include "lqg_single.h"


LQGSingle::LQGSingle()
{
 
}

LQGSingle::~LQGSingle()
{

} 

void LQGSingle::init(float a, float b, float k0, float ki, float f, float antiwindup)
{
    this->a  = a;
    this->b  = b;
    this->k0 = k0;
    this->ki = ki;
    this->f  = f;

    this->x_hat      = 0.0;

    this->integral_action = 0.0;


    this->antiwindup = antiwindup;
}

        
//xr  required velocity
//x   actual velocity
float LQGSingle::step(float xr, float x)
{
    //integral action
    //error = xr - x
    //integral_action+= ki@error * dt
    float error = xr - x;   
    float integral_action_new = integral_action + ki*error;


    //LQR controller with integral action
    //u = -k@x + ki@error_sum
    float u_new = -k0*x_hat + integral_action;

    //antiwindup with conditional integration
    float u = _clip(u_new, -antiwindup, antiwindup);

    integral_action = integral_action_new - (u_new - u);
    
    /*
    if (_abs(u_new - u) <= 10e-10)
    {
        integral_action = integral_action_new;
    }   
    */


    // kalman observer, prediction and correction
    float e         = x - x_hat;
    x_hat           = a*x_hat + b*u + f*e;

    return u;
}





float LQGSingle::_abs(float v)
{
    if (v < 0)
    {
        v = -v;
    }

    return v;
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


int LQGSingle::_sgn(float v)
{
    if (v > 0.0)
    {
        return 1;
    }
    else if (v < 0.0)
    {
        return -1;
    }
    else 
    {
        return 0;
    }
}