#ifndef _LQG_SINGLE_H_
#define _LQG_SINGLE_H_



class LQGSingle
{   
    public:
        LQGSingle();
        
        virtual ~LQGSingle();
        
        //k  : gain
        //ki : integral action gain
        void init(float a, float b, float k , float ki, float f, float antiwindup);
        
        //xr required value
        //x  actual value
        float step(float xr, float x);

        float get_x_hat();

    
    private:
        float _clip(float v, float min_v, float max_v);

       
    public:
        float a;
        float b;
        float k;
        float ki;
        float f;

        float x_hat;

        float integral_action;
        float antiwindup;
};

#endif
