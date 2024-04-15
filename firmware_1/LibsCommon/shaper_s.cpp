#include "shaper_s.h" 
#include <math_t.h>

void ShaperS::init(float ramp_up, float ramp_down, float alpha_up, float alpha_down)
{ 
    this->ramp_up   = ramp_up;
    this->ramp_down = ramp_down;
    this->alpha_up  = alpha_up;
    this->alpha_down  = alpha_down;

    this->x0 = 0.0; 
    this->x1 = 0.0;
}
 
 
float ShaperS::step(float x_req)
{
    float dif = x_req - this->x0;

    float ramp, alpha;

    if (dif >= 0) 
    {
        ramp  = this->ramp_up;
        alpha = this->alpha_up;
    }
    else
    {
        ramp  = this->ramp_down;
        alpha = this->alpha_down;
    }


    if (abs(dif) > ramp)
    {
        this->x0+= sgn(dif)*ramp;
    }
    else
    {
        this->x0 = x_req;
    }

   

    this->x1 = (1.0 - alpha)*this->x1 + alpha*this->x0;

    return this->x1;
}


void ShaperS::set(float value)
{
    this->x0 = value;
    this->x1 = value; 
}

