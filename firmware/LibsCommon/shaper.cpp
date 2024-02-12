#include "shaper.h"


void Shaper::init(float dx_p, float dx_n)
{
    this->dx_p = dx_p;
    this->dx_n = dx_n;

    this->x_curr = 0.0;
    this->saturated = false;
}

float Shaper::step(float x)
{
    // trapezoidal shaper
    float dif = x - this->x_curr;

    if (dif > this->dx_p)
    {
        this->x_curr+= this->dx_p;
    }
    else if (dif < this->dx_n)
    {
        this->x_curr+= this->dx_n;
    }
    else 
    {
        this->x_curr = x;
    }

    return this->x_curr;
}

bool Shaper::is_saturated()
{
    return this->saturated;
}
