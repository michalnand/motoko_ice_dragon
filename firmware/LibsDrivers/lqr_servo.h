#ifndef _LQR_SERVO_H_
#define _LQR_SERVO_H_



class LQRServo
{   
    public:
        LQRServo();
        
        virtual ~LQRServo();
        
        //k0 : position gain
        //k1 : velocity gain
        //ki : integral action gain
        void init(float k0, float k1, float ki, float antiwindup, float dt);
        

        //xr required position
        //x  actual position
        //dx actual velocity
        float step(float xr, float x, float dx);
        


    private:
        float _abs(float v);
        float _clip(float v, float min_v, float max_v);
       


    public:
        float k0;
        float k1;
        float ki;

        float integral_action;
        float antiwindup;
        float dt;
};

#endif
