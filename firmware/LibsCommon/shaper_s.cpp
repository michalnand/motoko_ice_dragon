#include "shaper_s.h" 
#include <math_t.h>

void ShaperS::init(float ramp, float alpha)
{
    this->ramp   = ramp;
    this->alpha  = 1.0 - alpha;

    this->x0 = 0.0;
    this->x1 = 0.0;
}
 
 
float ShaperS::step(float x_req)
{
    float dif = x_req - this->x0;

    if (abs(dif) > this->ramp)
    {
        this->x0+= this->ramp*sgn(dif);
    }
    else
    {
        this->x0 = x_req;
    }

    this->x1 = (1.0 - this->alpha)*this->x1 + this->alpha*this->x0;

    return this->x1;
}


