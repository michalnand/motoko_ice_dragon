#ifndef _LQR_SINGLE_H_
#define _LQR_SINGLE_H_



class LQRSingle
{   
    public:
        LQRSingle();
        
        virtual ~LQRSingle();
        
        //k0 : gain
        //ki : integral action gain
        void init(float k , float ki,  float antiwindup);
        
        //xr : required value
        //x  : actual value
        float step(float xr, float x); 
    
    private:
        float _clip(float v, float min_v, float max_v);

       
    public:
        float k;
        float ki;
        
        float integral_action;
        float antiwindup;
};

#endif
