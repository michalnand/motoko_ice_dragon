#ifndef _LQG_VELOCITY_H_
#define _LQG_VELOCITY_H_



class LQGVelocity
{   
    public:
        LQGVelocity();
        
        virtual ~LQGVelocity();
        
        //k0 : velocity gain
        //ki : integral action gain
        void init(float a, float b, float k0, float ki, float f, float antiwindup, float dt);
        

        //xr required velocity
        //x  actual velocity
        float step(float xr, float x);
        


    private:
        float _abs(float v);
        float _clip(float v, float min_v, float max_v);
       


    public:
        float a;
        float b;
        float k0;
        float ki;
        float f;

        float x_hat;

        float integral_action;
        float antiwindup;
        float dt;
};

#endif
