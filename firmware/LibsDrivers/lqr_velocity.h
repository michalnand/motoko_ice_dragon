#ifndef _LQR_VELOCITY_H_
#define _LQR_VELOCITY_H_



class LQRVelocity
{   
    public:
        LQRVelocity();
        
        virtual ~LQRVelocity();
        
        //k0 : velocity gain
        //ki : integral action gain
        void init(float k0, float ki, float antiwindup, float dt);
        

        //xr required velocity
        //x  actual velocity
        float step(float xr, float x);
        


    private:
        float _abs(float v);
        float _clip(float v, float min_v, float max_v);
       


    public:
        float k0;
        float ki;

        float integral_action;
        float antiwindup;
        float dt;
};

#endif
