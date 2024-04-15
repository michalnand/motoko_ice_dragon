#include "shaper.h"
#include <drivers.h>

void Shaper::init(float dx_p, float dx_n)
{
    this->dx_p = dx_p; 
    this->dx_n = dx_n;

    this->x_curr = 0.0;
    this->saturated = false; 
}

void Shaper::set_limits(float dx_p, float dx_n)
{
    this->dx_p = dx_p; 
    this->dx_n = dx_n;
}


float Shaper::step(float x)
{
    // trapezoidal shaper
    float dif = x - this->x_curr;  

    if (dif < this->dx_n)
    {
        this->x_curr+= this->dx_n;
        this->saturated = true;
    }
    else if (dif > this->dx_p)  
    {
        this->x_curr+= this->dx_p;
        this->saturated = true;
    }
    else  
    {
        this->x_curr = x;
        this->saturated = false;
    }

    return this->x_curr;
}



bool Shaper::is_saturated()
{
    return this->saturated;
}
