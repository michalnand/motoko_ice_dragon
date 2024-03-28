#ifndef _BIQUAD_FILTER_H_
#define _BIQUAD_FILTER_H_




class BiquadFilter
{
    public:
        BiquadFilter()
        {
            this->init(0, 0, 0, 0, 0);
        }

        void init(float b0, float b1, float b2, float a1, float a2)
        {
            this->b0 = b0;
            this->b1 = b1;
            this->b2 = b2;
            this->a1 = a1;
            this->a2 = a2;

            this->x1 = 0.0;
            this->x2 = 0.0;
            this->x0 = 0.0;

            this->y1 = 0.0;
            this->y2 = 0.0;
            this->y0 = 0.0;
        }

        float step(float x)
        {
            x2 = x1;
            x1 = x0;
            x0 = x;


            y2 = y1;
            y1 = y0;
            y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2;

            return y0;
        }


    private:
        float b0, b1, b2;
        float a1, a2;

        float x0, x1, x2;
        float y0, y1, y2;
        
};




#endif