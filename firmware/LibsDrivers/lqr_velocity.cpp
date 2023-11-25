#include "lqr_velocity.h"


LQRVelocity::LQRVelocity()
{

}

LQRVelocity::~LQRVelocity()
{

} 

void LQRVelocity::init(float k0, float ki, float antiwindup, float dt)
{
    this->k0 = k0;
    this->ki = ki;

    this->integral_action = 0.0;

    this->antiwindup = antiwindup;
    this->dt         = dt;
}

        
//xr  required velocity
//x   actual velocity
float LQRVelocity::step(float xr, float x)
{
    //integral action
    //error = xr - x
    //integral_action+= ki@error * dt
    float integral_action_new = this->integral_action + this->ki*(xr - x)*dt;


    //LQR controller with integral action
    //u = -k@x + ki@error_sum
    float u_new = -this->k0*x + this->integral_action;
 
    //antiwindup with conditional integration
    float u = _clip(u_new, -this->antiwindup, this->antiwindup);

    if (_abs(u_new - u) <= 10e-10)
    {
        this->integral_action = integral_action_new;
    }   

    return u;
}


float LQRVelocity::_abs(float v)
{
    if (v < 0)
    {
        v = -v;
    }

    return v;
}

float LQRVelocity::_clip(float v, float min_v, float max_v)
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
