#ifndef _SHAPER_H_
#define _SHAPER_H_


// trapezoidal signal shaper
// dx_p : maximum alloved x change when raising
// dx_n : maximum alloved x change when decreasing
class Shaper
{
    public:
        void init(float dx_p, float dx_n);
        void set_limits(float dx_p, float dx_n);

        float step(float x);
        bool is_saturated();

    private:
        float dx_p, dx_n, x_curr;
        bool saturated;
};



#endif