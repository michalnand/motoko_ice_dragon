#ifndef _SHAPER_S_H_
#define _SHAPER_S_H_


// s curve signal shaper
class ShaperS
{
    public:

        // ramp : maximum alloved change per step
        // alpha  : (0.0, 1.0), S-shape coeff
        // closer to 1 more S-like curve, 0 the shaper equals to trapezoidal
        void init(float ramp, float alpha = 0.5);
        float step(float x_req);
        void set(float value);

    private:
        float ramp, alpha;
        float x0, x1;
};


#endif