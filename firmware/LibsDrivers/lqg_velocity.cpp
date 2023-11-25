#include "lqg_velocity.h"


LQGVelocity::LQGVelocity()
{

}

LQGVelocity::~LQGVelocity()
{

} 

void LQGVelocity::init(float a, float b, float k0, float ki, float f, float antiwindup, float dt)
{
    this->a  = a;
    this->b  = b;
    this->k0 = k0;
    this->ki = ki;
    this->f  = f;

    this->x_hat      = 0.0;

    this->integral_action = 0.0;


    this->antiwindup = antiwindup;
    this->dt         = dt;
}

        
//xr  required velocity
//x   actual velocity
float LQGVelocity::step(float xr, float x)
{
    //integral action
    //error = xr - x
    //integral_action+= ki@error * dt
    float integral_action_new = integral_action + ki*(xr - x_hat)*dt;



    //LQR controller with integral action
    //u = -k@x + ki@error_sum
    float u_new = -k0*x_hat + integral_action;

    //antiwindup with conditional integration
    float u = _clip(u_new, -antiwindup, antiwindup);

    if (_abs(u_new - u) <= 10e-10)
    {
        integral_action = integral_action_new;
    }   

    // kalman observer, prediction and correction
    float e         = x - x_hat;
    float dx_hat    = a*x_hat + b*u + f*e;           
    x_hat           = x_hat + dx_hat*dt;

    return u;
}


float LQGVelocity::_abs(float v)
{
    if (v < 0)
    {
        v = -v;
    }

    return v;
}

float LQGVelocity::_clip(float v, float min_v, float max_v)
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
