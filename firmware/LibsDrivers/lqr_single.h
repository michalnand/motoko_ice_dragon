#ifndef _LQR_SINGLE_H_
#define _LQR_SINGLE_H_



class LQRSingle
{   
    public:
        LQRSingle();
        
        virtual ~LQRSingle();
        
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
