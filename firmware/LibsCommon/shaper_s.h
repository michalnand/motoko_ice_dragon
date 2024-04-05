#ifndef _SHAPER_S_H_
#define _SHAPER_S_H_


// s curve signal shaper
class ShaperS
{
    public:

        // ramp : maximum alloved change per step
        // alpha  : (0.0, 1.0), S-shape coeff
        // closer to 1 more S-like curve, 0 the shaper equals to trapezoidal
        void init(float ramp_up, float ramp_down, float alpha_up, float alpha_down);
        float step(float x_req);
        void set(float value);

    private:
        float ramp_up, ramp_down, alpha_up, alpha_down;
        float x0, x1;
};


#endif